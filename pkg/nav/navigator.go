package nav

import (
	"errors"
	"fmt"
	"math"
	"sort"
	"sync"

	"github.com/lisama569/nav-planner-go/pkg/grid"
	"github.com/lisama569/nav-planner-go/pkg/kinematics"
	"github.com/lisama569/nav-planner-go/pkg/local"
	"github.com/lisama569/nav-planner-go/pkg/planner"
)

// maxReplans 限定一次请求内“遇障→膨胀→重规划”的次数，保证有界、可复现。
const maxReplans = 4

// Status 是导航请求的结论分类。
type Status int

const (
	StatusUnknown             Status = iota
	StatusPlanned                    // 全局规划成功
	StatusReplanned                  // 遇障后重规划成功
	StatusRejected                   // 输入非法，详见 Reason
	StatusUnreachable                // 目标不可达
	StatusPlanLimit                  // 超出确定性展开预算
	StatusTimeout                    // 超出墙上时钟规划时限（实时兜底）
	StatusConstraintViolation        // 轨迹违反运动学约束
	StatusBlocked                    // 局部窗口堵死/终检仍碰撞且无替代路径
)

// String 返回状态的可读名称（用于日志与判定依据打印）。
func (s Status) String() string {
	switch s {
	case StatusPlanned:
		return "PLANNED"
	case StatusReplanned:
		return "REPLANNED"
	case StatusRejected:
		return "REJECTED"
	case StatusUnreachable:
		return "UNREACHABLE"
	case StatusPlanLimit:
		return "PLAN_LIMIT"
	case StatusTimeout:
		return "TIMEOUT"
	case StatusConstraintViolation:
		return "CONSTRAINT_VIOLATION"
	case StatusBlocked:
		return "BLOCKED"
	default:
		return "UNKNOWN"
	}
}

// Request 是一次规划/重规划请求。
type Request struct {
	Seq   int64
	Tick  int64
	Start kinematics.Pose
	Goal  grid.Cell
}

// Result 是可解释的规划结论。
type Result struct {
	ReqSeq     int64
	Tick       int64
	Status     Status
	Reason     error
	Path       []grid.Cell
	Traj       *kinematics.Trajectory
	Replans    int // 实际触发的重规划次数
	Expansions int // 全局搜索展开节点数（复现/诊断用）
	StartCell  grid.Cell
}

// MapUpdate 是带单调序号的地图更新。
type MapUpdate struct {
	Seq  int64
	Grid *grid.Grid
}

// ObstacleUpdate 是带单调序号的障碍物增删更新。
type ObstacleUpdate struct {
	Seq      int64
	Obstacle local.Obstacle
	Removed  bool
}

// Navigator 是线程安全的导航编排器。
//
// 并发模型：一把 RWMutex 保护地图快照、障碍物表与序号水位；
// 规划在锁外基于深拷贝快照进行（CPU 密集且不阻塞传感器更新）。
// 乱序规则：
//   - 地图更新按全局 Seq 单调，落后水位即拒绝（ErrStaleUpdate）；
//   - 障碍物更新按 (ID, Seq) 水位判定，同一 ID 的旧观测被丢弃，
//     不同 ID 的消息之间不强制全局排序。
type Navigator struct {
	mu sync.RWMutex
	gp *planner.Planner
	lp *local.Planner

	g      *grid.Grid
	mapSeq int64
	obs    map[string]local.Obstacle
	obsSeq map[string]int64

	// planPathFn 为测试专用接缝：非 nil 时替代 A* 搜索（模拟规划器/轨迹
	// 构造器输出与保守膨胀不一致的漂移），生产代码恒为 nil，行为不变。
	planPathFn func(g *grid.Grid, start, goal grid.Cell) ([]grid.Cell, error)
}

// New 创建编排器，初始地图可为 nil（后续由 ApplyMap 注入）。
func New(gp *planner.Planner, lp *local.Planner, g *grid.Grid) *Navigator {
	return &Navigator{
		gp:     gp,
		lp:     lp,
		g:      g,
		obs:    map[string]local.Obstacle{},
		obsSeq: map[string]int64{},
	}
}

// ApplyMap 原子替换地图；乱序（Seq 落后）更新被拒绝。
func (n *Navigator) ApplyMap(u MapUpdate) error {
	if u.Grid == nil {
		return ErrInvalidUpdate
	}
	n.mu.Lock()
	defer n.mu.Unlock()
	if u.Seq <= n.mapSeq && n.g != nil {
		return fmt.Errorf("%w: map seq=%d <= applied=%d", ErrStaleUpdate, u.Seq, n.mapSeq)
	}
	n.g = u.Grid.Clone()
	n.mapSeq = u.Seq
	return nil
}

// ApplyObstacle 原子增删障碍物；乱序更新按 (障碍物ID, Seq) 水位丢弃。
func (n *Navigator) ApplyObstacle(u ObstacleUpdate) error {
	id := u.Obstacle.ID
	n.mu.Lock()
	defer n.mu.Unlock()
	if cur := n.obsSeq[id]; u.Seq <= cur {
		return fmt.Errorf("%w: obstacle %q seq=%d <= applied=%d", ErrStaleUpdate, id, u.Seq, cur)
	}
	n.obsSeq[id] = u.Seq
	if u.Removed {
		delete(n.obs, id)
	} else {
		n.obs[id] = u.Obstacle
	}
	return nil
}

// snapshotLocked 调用方需持锁；返回深拷贝地图与按 ID 排序的障碍物切片。
func (n *Navigator) snapshot() (*grid.Grid, []local.Obstacle) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	g := n.g.Clone()
	ids := make([]string, 0, len(n.obs))
	for id := range n.obs {
		ids = append(ids, id)
	}
	sort.Strings(ids)
	out := make([]local.Obstacle, 0, len(ids))
	for _, id := range ids {
		out = append(out, n.obs[id])
	}
	return g, out
}

// Snapshot 返回当前地图与障碍物的深拷贝（一致读），障碍物按 ID 排序。
func (n *Navigator) Snapshot() (*grid.Grid, []local.Obstacle) {
	return n.snapshot()
}

// Plan 执行一次性的完整路线请求。
//
// 它与 ReplanningStep 共用同一条规划闭环和同一份世界快照：已通过更新被接受的
// 动态障碍物一律计入（先在原始静态地图上搜索，再用整条轨迹对所有已知障碍终检，
// 冲突则膨胀障碍足迹后重规划）。区别仅在于 Plan 不跑 DWA 逐步执行校验，因此
// 成功/绕行/不可达/拒绝/超时/预算等结论与路线在两个入口间可互相解释。
func (n *Navigator) Plan(req Request) Result {
	return n.runClosedLoop(req, false)
}

// ReplanningStep 在当前地图+障碍物状态下执行“规划—终检—局部执行—遇障重规划”
// 闭环。与 Plan 使用相同的世界快照与终检闸门，额外增加 DWA 逐步可执行性校验：
// 局部窗口被堵时同样膨胀障碍足迹后重新全局搜索。
// 成功时若发生过绕行，状态为 REPLANNED（否则 PLANNED）。
func (n *Navigator) ReplanningStep(req Request) Result {
	return n.runClosedLoop(req, true)
}

// runClosedLoop 是 Plan 与 ReplanningStep 共享的确定性规划闭环。
//
//  1. 基于同一快照（原始地图 + 全部已接受障碍）做一次静态规划；
//  2. 对“准备下发的整条轨迹”做终检：逐段复核原始地图与所有动态障碍；
//  3. 逐步入口（rollout=true）再跑 DWA 短步推进验证可执行性；
//  4. 任一闸门发现冲突：把障碍足迹与碰撞点单元格确定性膨胀进工作地图，重搜；
//  5. 终检仍碰撞且膨胀不再带来新信息（或重搜次数耗尽）=> BLOCKED，
//     绝不下发穿过已知障碍的轨迹；其他失败分类（拒绝/不可达/预算/超时）原样透传。
func (n *Navigator) runClosedLoop(req Request, rollout bool) Result {
	g, obstacles := n.snapshot()
	if g == nil {
		// 输入分类在 planOnce 内统一完成，保持两个入口的拒绝结论一致。
		return n.planOnce(req, nil, obstacles, 0)
	}
	margin := 0.0
	if n.lp != nil {
		margin = n.lp.Config().ObstacleMargin
	}
	work := g.Clone()
	var lastRes Result
	replans := 0
	for attempt := 0; attempt <= maxReplans; attempt++ {
		res := n.planOnce(req, work, obstacles, replans)
		lastRes = res
		if res.Status != StatusPlanned {
			// 终检/执行触发的重规划若以“不可达”告终，保留 UNREACHABLE 分类
			//（兼容既有语义，两个入口结论一致）；起点被膨胀封死仍为 BLOCKED。
			return res
		}

		// 闸门 1（两个入口共有）：整条交付轨迹对原始静态地图与当前所有
		// 已知动态障碍逐段终检，而不只看 DWA 局部试探的那一小段。
		hit, col, _ := n.lp.FirstCollision(res.Traj, g, obstacles)
		// 闸门 2（仅逐步入口）：DWA 短步推进的可执行性校验。
		blocked := false
		var blockerCells []grid.Cell
		if !hit && rollout {
			blocked, blockerCells = n.localRollout(req.Start, res.Path, work, obstacles)
		}
		if !hit && !blocked {
			res.Status = statusForAttempt(replans)
			res.Replans = replans
			return res
		}

		// 发现冲突：膨胀障碍足迹（终检碰撞时额外标记碰撞点所在格），然后重搜。
		newCells := blockerCells
		if hit {
			newCells = append(newCells, colCell(col))
			newCells = append(newCells, obstacleFootprintCells(g, obstacles, margin)...)
		}
		added := inflateObstacles(work, dedupCells(newCells))
		if attempt == maxReplans {
			break
		}
		// 膨胀没有带来任何新的禁行格：再搜一次结论也不会变，直接判死。
		if added == 0 {
			break
		}
		replans++
	}
	// 用尽重规划次数，最后一次规划“看起来成功”但终检/执行闸门仍未通过：
	// 判 BLOCKED（带可区分哨兵原因），清掉半成品路线与轨迹。
	if lastRes.Status == StatusPlanned {
		lastRes = blockedCollisionResult(lastRes, replans)
	}
	return lastRes
}

// blockedCollisionResult 把“规划看似成功但终检仍碰撞”的结论改写为
// BLOCKED + ErrTrajectoryCollision，并清除不得下发的半成品路线/轨迹。
func blockedCollisionResult(res Result, replans int) Result {
	res.Status = StatusBlocked
	res.Reason = fmt.Errorf("%w: trajectory still collides after %d replans",
		ErrTrajectoryCollision, replans)
	res.Replans = replans
	res.Path = nil
	res.Traj = nil
	return res
}

// colCell 取终检碰撞段所在栅格（与 local 碰撞检测的取整口径一致）。
func colCell(col local.Collision) grid.Cell {
	return grid.Cell{
		X: int(math.Floor(col.Pose.X + 0.5)),
		Y: int(math.Floor(col.Pose.Y + 0.5)),
	}
}

// dedupCells 对单元格切片去重（顺序无关的确定性集合输入）。
func dedupCells(cells []grid.Cell) []grid.Cell {
	seen := map[grid.Cell]bool{}
	out := cells[:0]
	for _, c := range cells {
		if seen[c] {
			continue
		}
		seen[c] = true
		out = append(out, c)
	}
	return out
}

func statusForAttempt(replans int) Status {
	if replans > 0 {
		return StatusReplanned
	}
	return StatusPlanned
}

// planOnce 在给定工作地图上完成一次完整的静态规划与轨迹校验。
func (n *Navigator) planOnce(req Request, g *grid.Grid, obstacles []local.Obstacle, replans int) Result {
	res := Result{ReqSeq: req.Seq, Tick: req.Tick, Replans: replans}
	if n.gp == nil || n.lp == nil {
		res.Status = StatusRejected
		res.Reason = ErrNoPlanner
		return res
	}
	if g == nil {
		res.Status = StatusRejected
		res.Reason = ErrMapMissing
		return res
	}
	startCell := poseToCell(req.Start)
	res.StartCell = startCell

	var path []grid.Cell
	var err error
	if n.planPathFn != nil {
		path, err = n.planPathFn(g, startCell, req.Goal)
	} else {
		path, err = n.gp.Plan(g, startCell, req.Goal)
		res.Expansions = n.gp.Expansions()
	}
	if err != nil {
		res.Status, res.Reason = classifyPlanError(err, replans)
		return res
	}
	res.Path = path

	traj, err := kinematics.PathToTrajectory(path, req.Start, n.lp.Constraints(), n.lp.DT())
	if err != nil {
		res.Status = StatusRejected
		res.Reason = err
		return res
	}
	if vs := kinematics.Validate(traj, req.Start, n.lp.Constraints()); len(vs) > 0 {
		res.Status = StatusConstraintViolation
		res.Reason = fmt.Errorf("%w: %s", ErrConstraintViolation, vs[0].Error())
		res.Traj = traj
		return res
	}
	res.Traj = traj
	res.Status = StatusPlanned
	return res
}

// localRollout 沿 path 用 DWA 短步推进；遇到堵点返回 blocked=true 及应膨胀的单元格。
// 注意：它只做“可执行性”判定，不修改轨迹（最终下发轨迹由全局路径构造）。
func (n *Navigator) localRollout(start kinematics.Pose, path []grid.Cell, g *grid.Grid, obstacles []local.Obstacle) (bool, []grid.Cell) {
	cur := start
	goal := path[len(path)-1]
	for it := 0; it < n.lp.MaxIters(); it++ {
		if n.lp.Arrived(cur, goal) {
			return false, nil
		}
		traj, blocked, err := n.lp.Step(cur, path, g, obstacles)
		if err != nil {
			return true, nil
		}
		if blocked || n.lp.Collides(traj, g, obstacles) {
			return true, obstacleFootprintCells(g, obstacles, n.lp.Config().ObstacleMargin)
		}
		if len(traj.Points) == 0 {
			return true, nil
		}
		cur = traj.End()
	}
	// 达到局部推进迭代上限仍未到目标：视为堵死，触发重规划。
	return true, obstacleFootprintCells(g, obstacles, n.lp.Config().ObstacleMargin)
}

// poseToCell 用四舍五入把连续位姿映射到其所在栅格（CellWorld 的逆映射约定）。
func poseToCell(p kinematics.Pose) grid.Cell {
	return grid.Cell{X: int(math.Round(p.X)), Y: int(math.Round(p.Y))}
}

// classifyPlanError 把规划器错误映射为可区分的导航状态。
func classifyPlanError(err error, replans int) (Status, error) {
	switch {
	case errors.Is(err, planner.ErrUnreachable):
		return StatusUnreachable, err
	case errors.Is(err, planner.ErrPlanLimit):
		return StatusPlanLimit, err
	case errors.Is(err, planner.ErrPlanTimeout):
		return StatusTimeout, err
	default:
		// 地图缺失、起终点越界/占据等一律是输入拒绝。
		// 重规划中起点被膨胀障碍占据意味着机器人已被封死，归类 Blocked。
		if replans > 0 && errors.Is(err, planner.ErrStartOccupied) {
			return StatusBlocked, err
		}
		return StatusRejected, err
	}
}

// inflateObstacles 在工作地图上标记障碍物足迹单元格（确定性集合更新），
// 返回本次新被标记（原先自由）的单元格数，用于判断重规划是否还有意义。
func inflateObstacles(g *grid.Grid, cells []grid.Cell) int {
	added := 0
	for _, c := range cells {
		if g.InBounds(c) && !g.Occupied(c) {
			g.Set(c, true)
			added++
		}
	}
	return added
}

// obstacleFootprintCells 计算所有圆形障碍（含安全裕度）覆盖的单元格，
// 结果按 (Y,X) 排序去重，保证确定性。
func obstacleFootprintCells(g *grid.Grid, obstacles []local.Obstacle, margin float64) []grid.Cell {
	seen := map[grid.Cell]bool{}
	for _, o := range obstacles {
		r := o.Radius + margin
		ri := int(math.Ceil(r))
		for x := int(math.Floor(o.X)) - ri; x <= int(math.Ceil(o.X))+ri; x++ {
			for y := int(math.Floor(o.Y)) - ri; y <= int(math.Ceil(o.Y))+ri; y++ {
				c := grid.Cell{X: x, Y: y}
				if !g.InBounds(c) {
					continue
				}
				wx, wy := kinematics.CellWorld(c)
				if math.Hypot(wx-o.X, wy-o.Y) <= r {
					seen[c] = true
				}
			}
		}
	}
	out := make([]grid.Cell, 0, len(seen))
	for c := range seen {
		out = append(out, c)
	}
	sort.Slice(out, func(i, j int) bool {
		if out[i].Y != out[j].Y {
			return out[i].Y < out[j].Y
		}
		return out[i].X < out[j].X
	})
	return out
}
