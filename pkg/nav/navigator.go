package nav

import (
	"errors"
	"fmt"
	"math"
	"sort"
	"sync"
	"time"

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
	StatusPlanLimit                  // 超出规划预算/时限
	StatusConstraintViolation        // 轨迹违反运动学约束
	StatusBlocked                    // 局部窗口堵死且无替代路径
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

// Options 是编排器的可选实时兜底配置；零值表示完全确定性模式（默认）。
type Options struct {
	// PlanTimeout 是单次规划请求的墙钟兜底时限，用于兜住超大/极端地图下的
	// 最坏耗时；<=0 表示禁用（默认）。
	//
	// 边界说明：确定性保证（同输入必同结论、可重放）建立在“无墙上时钟”上，
	// 由展开预算（planner 的 maxExpansions）充当确定性的规划时限。启用
	// PlanTimeout 后，超时是否触发与机器负载相关，跨机器重放可能在该点分叉；
	// 但无论何时触发，结论都明确分类为 StatusPlanLimit/ErrPlanTimeout，
	// 且绝不下发半成品路径或轨迹。需要严格重放的场景请保持默认（禁用）。
	PlanTimeout time.Duration
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
	opts   Options
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

// SetOptions 原子更新可选配置（如墙钟兜底时限）；零值恢复完全确定性模式。
func (n *Navigator) SetOptions(o Options) {
	n.mu.Lock()
	defer n.mu.Unlock()
	n.opts = o
}

// options 读取当前配置（值拷贝，一致读）。
func (n *Navigator) options() Options {
	n.mu.RLock()
	defer n.mu.RUnlock()
	return n.opts
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

// Plan 执行一次性全局规划：输入分类 → A* → 轨迹构造 → 约束校验 → 动态障碍终检。
//
// 与 ReplanningStep 共享同一“世界观”和同一规划闭环：两者都基于同一份
// 地图+障碍物快照，都会把已接受的动态障碍计入结论——交付轨迹若与任一已知
// 动态障碍冲突，就把冲突单元格与障碍足迹确定性膨胀进工作地图后重规划
// （有界，maxReplans 次），仍冲突则明确判 BLOCKED，绝不下发穿障轨迹。
// 区别仅在于 Plan 不做 DWA 局部可执行性滚校（那是 ReplanningStep 的职责），
// 因此同一请求下两者的成功/绕行/不可达/被拒绝结论可以互相解释。
func (n *Navigator) Plan(req Request) Result {
	g, obstacles := n.snapshot()
	return n.planLoop(req, g, obstacles, false, n.deadline())
}

// ReplanningStep 在当前地图+障碍物状态下执行“规划—终检—局部执行—遇障重规划”闭环：
// 交付轨迹先整体对当前已知动态障碍复核（终检），再用 DWA 沿路径短步推进做
// 可执行性验证；任一环节发现冲突，就把冲突单元格/障碍足迹确定性膨胀到工作地图，
// 然后重新全局搜索；成功时返回 StatusReplanned（若根本没触发重规划则为 Planned）。
func (n *Navigator) ReplanningStep(req Request) Result {
	g, obstacles := n.snapshot()
	return n.planLoop(req, g, obstacles, true, n.deadline())
}

// deadline 根据可选配置计算本次请求的墙钟兜底截止点；零值表示不启用。
func (n *Navigator) deadline() time.Time {
	if to := n.options().PlanTimeout; to > 0 {
		return time.Now().Add(to)
	}
	return time.Time{}
}

// timeoutResult 构造“超时/未完成”类结论：明确分类、绝不下发半成品。
func timeoutResult(req Request, replans int) Result {
	return Result{
		ReqSeq: req.Seq, Tick: req.Tick, Replans: replans,
		Status: StatusPlanLimit, Reason: ErrPlanTimeout,
		StartCell: poseToCell(req.Start),
	}
}

// planLoop 是 Plan 与 ReplanningStep 共用的“规划—终检—膨胀重规划”闭环，
// 保证两种发起方式对同一时刻的世界看法一致、结论分类一致。
// useLocalRollout=true 时（ReplanningStep）额外做 DWA 局部可执行性验证。
//
// 失败结论（BLOCKED 等）一律清空 Path/Traj：半成品不得被当作有效结果下发。
func (n *Navigator) planLoop(req Request, g *grid.Grid, obstacles []local.Obstacle, useLocalRollout bool, deadline time.Time) Result {
	work := g.Clone() // nil 安全：Clone(nil)=nil，planOnce 会分类为 REJECTED
	var lastRes Result
	replans := 0
	for attempt := 0; attempt <= maxReplans; attempt++ {
		if deadlineExceeded(deadline) {
			return timeoutResult(req, replans)
		}
		res := n.planOnce(req, work, obstacles, replans)
		res = reclassifyDynamicBlock(res, req, g)
		lastRes = res
		if res.Status != StatusPlanned {
			return res
		}
		// 终检：最终交付的整条轨迹必须对当前所有已知动态障碍（与真实地图）
		// 复核一遍；局部能绕开但交付轨迹仍撞上的，继续膨胀重规划或判失败。
		if cells := n.collidingCells(res.Traj, g, obstacles); len(cells) > 0 {
			if attempt == maxReplans {
				break
			}
			inflateObstacles(work, cells)
			inflateObstacles(work, obstacleFootprintCells(work, obstacles, n.lp.Config().ObstacleMargin))
			replans++
			continue
		}
		if useLocalRollout {
			// 局部执行校验：沿全局路径逐步滚 DWA，探测动态障碍冲突。
			blocked, blockerCells := n.localRollout(req.Start, res.Path, work, obstacles, deadline)
			if deadlineExceeded(deadline) {
				return timeoutResult(req, replans)
			}
			if blocked {
				if attempt == maxReplans {
					break
				}
				// 重规划：把堵点障碍膨胀进工作地图后再搜一次。
				inflateObstacles(work, blockerCells)
				replans++
				continue
			}
		}
		res.Status = statusForAttempt(replans)
		res.Replans = replans
		return res
	}
	// 失败结论不得携带路径/轨迹，避免半成品被误下发。
	lastRes.Status = StatusBlocked
	lastRes.Reason = fmt.Errorf("nav: no collision-free trajectory after %d replans", replans)
	lastRes.Path = nil
	lastRes.Traj = nil
	lastRes.Replans = replans
	return lastRes
}

// reclassifyDynamicBlock 把“仅因动态障碍膨胀导致的起/终点占据”从输入拒绝
// 重分类为可解释的结论：目标被动态障碍覆盖 → UNREACHABLE（目标本身合法），
// 起点被动态障碍封死 → BLOCKED（机器人被围困，非输入非法）。
// 真实地图上本就占据的单元格不受影响，仍按输入非法 REJECTED。
func reclassifyDynamicBlock(res Result, req Request, g *grid.Grid) Result {
	if res.Status != StatusRejected || g == nil {
		return res
	}
	switch {
	case errors.Is(res.Reason, planner.ErrGoalOccupied) && g.Free(req.Goal):
		res.Status = StatusUnreachable
		res.Reason = fmt.Errorf("nav: goal %v covered by dynamic obstacle: %w", req.Goal, planner.ErrUnreachable)
	case errors.Is(res.Reason, planner.ErrStartOccupied) && g.Free(res.StartCell):
		res.Status = StatusBlocked
		res.Reason = fmt.Errorf("nav: start %v surrounded by dynamic obstacle: %w", res.StartCell, planner.ErrStartOccupied)
	}
	return res
}

// collidingCells 返回交付轨迹与当前已知动态障碍（或真实地图占据格）冲突的
// 单元格集合，按 (Y,X) 排序去重保证确定性；空切片表示终检通过。
func (n *Navigator) collidingCells(traj *kinematics.Trajectory, g *grid.Grid, obstacles []local.Obstacle) []grid.Cell {
	if traj == nil || n.lp == nil {
		return nil
	}
	seen := map[grid.Cell]bool{}
	for _, pt := range traj.Points {
		cell := grid.Cell{X: int(math.Floor(pt.Pose.X + 0.5)), Y: int(math.Floor(pt.Pose.Y + 0.5))}
		if n.lp.CollidesAt(pt.Pose, obstacles) || (g != nil && !g.Free(cell)) {
			seen[cell] = true
		}
	}
	if len(seen) == 0 {
		return nil
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

	path, err := n.gp.Plan(g, startCell, req.Goal)
	res.Expansions = n.gp.Expansions()
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
// 启用墙钟兜底时每次迭代检查一次截止点，超时即提前返回（由调用方分类）。
func (n *Navigator) localRollout(start kinematics.Pose, path []grid.Cell, g *grid.Grid, obstacles []local.Obstacle, deadline time.Time) (bool, []grid.Cell) {
	cur := start
	goal := path[len(path)-1]
	for it := 0; it < n.lp.MaxIters(); it++ {
		if deadlineExceeded(deadline) {
			return true, nil
		}
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

// deadlineExceeded 判定墙钟兜底是否触发；零值截止点表示未启用（确定性模式）。
func deadlineExceeded(d time.Time) bool {
	return !d.IsZero() && time.Now().After(d)
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
	default:
		// 地图缺失、起终点越界/占据等一律是输入拒绝。
		// 重规划中起点被膨胀障碍占据意味着机器人已被封死，归类 Blocked。
		if replans > 0 && errors.Is(err, planner.ErrStartOccupied) {
			return StatusBlocked, err
		}
		return StatusRejected, err
	}
}

// inflateObstacles 在工作地图上标记障碍物足迹单元格（确定性集合更新）。
func inflateObstacles(g *grid.Grid, cells []grid.Cell) {
	for _, c := range cells {
		if g.InBounds(c) {
			g.Set(c, true)
		}
	}
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
