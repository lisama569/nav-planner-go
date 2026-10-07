// Package local 实现基于动态窗口法（DWA）的确定性局部避障。
//
// 策略概述：
//   - 在约束窗口内确定性枚举 (V, Omega) 指令（含原地转向与停车），
//     线速度从高到低、角速度从小到大遍历，固定采样顺序即固定平局裁决；
//   - 每条候选指令用差速运动学（kinematics.Integrate）精确前滚 Horizon 步；
//   - 任何一步进入圆形障碍物（半径+安全裕度）或越界栅格即淘汰；
//   - 对幸存候选按“末端到目标距离 + 朝向偏差 + 间隙裕度”加权打分，取最优；
//   - 没有任何幸存候选时返回 blocked=true，由 nav 层触发全局重规划。
//
// 输出轨迹由精确积分生成，因此天然满足运动学约束，nav 层仍会二次 Validate。
package local

import (
	"fmt"
	"math"

	"github.com/lisama569/nav-planner-go/pkg/grid"
	"github.com/lisama569/nav-planner-go/pkg/kinematics"
)

// Obstacle 是圆形动态障碍物，Seq 为传感器单调序号（乱序治理在 nav 层）。
type Obstacle struct {
	ID     string
	X, Y   float64
	Radius float64
	Seq    int64
}

// Config 为局部规划器参数。
type Config struct {
	DT              float64 // 仿真步长（秒）
	Horizon         int     // 前视步数
	VSamples        int     // 线速度采样数
	OmegaSamples    int     // 角速度采样数（含 0，采样在两侧对称展开）
	GoalTolerance   float64 // 到达目标容差（栅格单位）
	ObstacleMargin  float64 // 额外安全裕度（栅格单位）
	MaxIters        int     // nav 层推进到目标的最大迭代数
	HeadingWeight   float64 // 朝向偏差评分权重
	ClearanceWeight float64 // 间隙评分权重
}

// Planner 是确定性 DWA 局部规划器。
type Planner struct {
	cfg      Config
	cons     kinematics.Constraints
	omegaMax float64 // 由 MaxOmega 与 MaxDTheta/DT 联合收紧
}

// New 创建局部规划器并固化参数（非法配置返回 nil，错误由 nav 组装时避免）。
func New(cfg Config, cons kinematics.Constraints) *Planner {
	if cfg.DT <= 0 || cfg.Horizon <= 0 || cfg.VSamples <= 0 ||
		cfg.OmegaSamples <= 0 || cfg.GoalTolerance <= 0 || cfg.MaxIters <= 0 {
		return nil
	}
	if cons.MaxV <= 0 || cons.MaxOmega <= 0 || cons.MaxDTheta <= 0 || cons.MaxCurvature <= 0 {
		return nil
	}
	om := cons.MaxOmega
	if maxByTurn := cons.MaxDTheta / cfg.DT; maxByTurn < om {
		om = maxByTurn
	}
	if cfg.HeadingWeight == 0 && cfg.ClearanceWeight == 0 {
		cfg.HeadingWeight = 0.2
		cfg.ClearanceWeight = 0.05
	}
	return &Planner{cfg: cfg, cons: cons, omegaMax: om}
}

// Arrived 判断当前位姿是否已进入目标单元格的到达容差范围。
func (p *Planner) Arrived(pose kinematics.Pose, goal grid.Cell) bool {
	gx, gy := kinematics.CellWorld(goal)
	return math.Hypot(pose.X-gx, pose.Y-gy) <= p.cfg.GoalTolerance
}

// Step 从 start 出发沿 path（全局路径，含起终点）滚出一段无碰轨迹
// （固定 DT、共 Horizon 段）。采用“胡萝卜点”跟随：只朝当前前视的最近路径
// 单元格转向，避免 DWA 斜切墙角把自己送入死位。
// blocked=true 表示局部窗口被堵死（除停车外无合法指令），调用方应触发重规划。
func (p *Planner) Step(start kinematics.Pose, path []grid.Cell, g *grid.Grid, obstacles []Obstacle) (traj *kinematics.Trajectory, blocked bool, err error) {
	if g == nil {
		return nil, false, ErrNoMap
	}
	if len(path) < 2 {
		return nil, false, ErrBadConfig
	}
	aim := p.carrot(start, path)
	gx, gy := kinematics.CellWorld(aim)

	var best *candidate
	haveBest := false

	// 固定枚举顺序：v 从高到低；每个 v 下 omega 从 -omegaMax 到 +omegaMax。
	// 原地转向（v=0, omega!=0）作为最末一组兜底；停车指令不参与，
	// 防止机器人把“原地不动”误判为最优而永久停滞。
	for vi := 0; vi < p.cfg.VSamples; vi++ {
		v := p.cons.MaxV * float64(p.cfg.VSamples-vi) / float64(p.cfg.VSamples)
		p.evalOmega(start, gx, gy, g, obstacles, v, &best, &haveBest)
	}
	p.evalOmega(start, gx, gy, g, obstacles, 0, &best, &haveBest)

	if !haveBest {
		return nil, true, nil
	}

	// 用最优指令精确前滚，生成保证运动学一致的轨迹。
	traj = &kinematics.Trajectory{Points: make([]kinematics.Point, 0, p.cfg.Horizon)}
	cur := start
	minClear := math.Inf(1)
	for k := 0; k < p.cfg.Horizon; k++ {
		nxt := kinematics.Integrate(cur, best.v, best.omega, p.cfg.DT)
		traj.Points = append(traj.Points, kinematics.Point{
			Pose: nxt, V: best.v, Omega: best.omega, DT: p.cfg.DT,
		})
		cur = nxt
		if c := p.clearance(cur, obstacles); c < minClear {
			minClear = c
		}
	}
	return traj, false, nil
}

// evalOmega 枚举某个 v 下的全部 omega 采样并更新最优候选。
// candidate 是一条 DWA 候选指令及其评分。
type candidate struct {
	v, omega float64
	score    float64
}

func (p *Planner) evalOmega(start kinematics.Pose, gx, gy float64, g *grid.Grid, obstacles []Obstacle, v float64, best **candidate, haveBest *bool) {
	for oi := 0; oi < p.cfg.OmegaSamples; oi++ {
		var omega float64
		if p.cfg.OmegaSamples == 1 {
			omega = 0
		} else {
			omega = p.omegaMax * (2*float64(oi)/float64(p.cfg.OmegaSamples-1) - 1)
		}
		// 曲率约束：行进段 |omega/v| <= MaxCurvature
		if math.Abs(v) > 1e-12 && math.Abs(omega/v) > p.cons.MaxCurvature+1e-9 {
			continue
		}
		// 原地转向组中剔除纯停车指令（仅在全部行进/转向候选失败时由调用方判定堵死）。
		if math.Abs(v) <= 1e-12 && math.Abs(omega) <= 1e-12 {
			continue
		}
		if math.Abs(omega) > p.omegaMax+1e-9 {
			continue
		}
		// 前滚并做碰撞检测
		cur := start
		minClear := math.Inf(1)
		collide := false
		for k := 0; k < p.cfg.Horizon; k++ {
			cur = kinematics.Integrate(cur, v, omega, p.cfg.DT)
			cell := grid.Cell{X: int(math.Floor(cur.X + 0.5)), Y: int(math.Floor(cur.Y + 0.5))}
			if !g.Free(cell) {
				collide = true
				break
			}
			c := p.clearance(cur, obstacles)
			if c < minClear {
				minClear = c
			}
			if c < 0 {
				collide = true
				break
			}
		}
		if collide {
			continue
		}
		// 评分：末端距离 + 朝向偏差 + 间隙裕度
		dist := math.Hypot(cur.X-gx, cur.Y-gy)
		headingToGoal := math.Atan2(gy-cur.Y, gx-cur.X)
		dh := math.Abs(wrapPi(headingToGoal - cur.Theta))
		score := dist + p.cfg.HeadingWeight*dh - p.cfg.ClearanceWeight*math.Min(minClear, 3.0)
		c := candidate{v: v, omega: omega, score: score}
		// 严格小于才替换：枚举顺序固定 => 平局选择可复现。
		if !*haveBest || score < (*best).score {
			cc := c
			*best = &cc
			*haveBest = true
		}
	}
}

// carrot 在前视半径内选择路径上距当前位姿最远的可达单元格作为局部瞄准点，
// 保证局部执行始终贴着全局路径走（含初始朝向修正的平滑逼近）。
func (p *Planner) carrot(start kinematics.Pose, path []grid.Cell) grid.Cell {
	// 前视半径至少一个栅格，避免瞄准点停在自身所在格造成原地摇摆。
	lookahead := p.cfg.DT * p.cons.MaxV * float64(p.cfg.Horizon) * 0.6
	if lookahead < 1.0 {
		lookahead = 1.0
	}
	// 找最近的路径点下标
	bestIdx := 0
	bestD := math.Inf(1)
	for i, c := range path {
		cx, cy := kinematics.CellWorld(c)
		if d := math.Hypot(start.X-cx, start.Y-cy); d < bestD {
			bestD, bestIdx = d, i
		}
	}
	aim := bestIdx
	for aim+1 < len(path) {
		cx, cy := kinematics.CellWorld(path[aim+1])
		if math.Hypot(start.X-cx, start.Y-cy) > lookahead {
			break
		}
		aim++
	}
	// 尚未到终点时，至少瞄准下一个路径点。
	if aim == bestIdx && bestIdx+1 < len(path) {
		aim = bestIdx + 1
	}
	return path[aim]
}

// Collides 判断给定轨迹是否与任一障碍物（或占据栅格）碰撞。
func (p *Planner) Collides(traj *kinematics.Trajectory, g *grid.Grid, obstacles []Obstacle) bool {
	hit, _, _ := p.FirstCollision(traj, g, obstacles)
	return hit
}

// Collision 描述终检发现的首个碰撞点（段号、位姿与原因），用于日志/判定依据。
type Collision struct {
	Index int
	Pose  kinematics.Pose
	Msg   string
}

// FirstCollision 对整条轨迹逐点做碰撞终检，返回首个碰撞点（固定从首段起，
// 结论确定）。检测口径与 DWA 局部试探完全一致：
//   - 圆形障碍：到障碍中心距离 < 半径+ObstacleMargin；
//   - 栅格：采样点所在单元格（四舍五入）被占据或越界。
//
// 无碰撞时 hit=false。终检覆盖整条轨迹，而不只是局部窗口的一小段。
func (p *Planner) FirstCollision(traj *kinematics.Trajectory, g *grid.Grid, obstacles []Obstacle) (hit bool, col Collision, collider string) {
	if traj == nil {
		return false, Collision{}, ""
	}
	for i, pt := range traj.Points {
		if id, d := p.nearestIntruder(pt.Pose, obstacles); id != "" {
			return true, Collision{
				Index: i, Pose: pt.Pose,
				Msg: fmt.Sprintf("segment %d pose (%.3f,%.3f) intrudes obstacle %q by %.3f",
					i, pt.Pose.X, pt.Pose.Y, id, -d),
			}, id
		}
		cell := grid.Cell{X: int(math.Floor(pt.Pose.X + 0.5)), Y: int(math.Floor(pt.Pose.Y + 0.5))}
		if g == nil || !g.Free(cell) {
			reason := "occupied/out-of-bounds cell"
			if g == nil {
				reason = "map missing"
			}
			return true, Collision{
				Index: i, Pose: pt.Pose,
				Msg: fmt.Sprintf("segment %d pose (%.3f,%.3f) enters %s %v",
					i, pt.Pose.X, pt.Pose.Y, reason, cell),
			}, "grid:" + cell.String()
		}
	}
	return false, Collision{}, ""
}

// nearestIntruder 返回侵入安全裕度最多的障碍物 ID 与侵入量（负=侵入）。
func (p *Planner) nearestIntruder(pose kinematics.Pose, obstacles []Obstacle) (string, float64) {
	worstID := ""
	worst := 0.0
	for _, o := range obstacles {
		d := math.Hypot(pose.X-o.X, pose.Y-o.Y) - o.Radius - p.cfg.ObstacleMargin
		if d < worst {
			worst, worstID = d, o.ID
		}
	}
	return worstID, worst
}

// clearance 返回位姿到最近障碍物表面（已含配置的安全裕度）的距离；负值代表侵入。
func (p *Planner) clearance(pose kinematics.Pose, obstacles []Obstacle) float64 {
	best := math.Inf(1)
	for _, o := range obstacles {
		d := math.Hypot(pose.X-o.X, pose.Y-o.Y) - o.Radius - p.cfg.ObstacleMargin
		if d < best {
			best = d
		}
	}
	return best
}

// wrapPi 把角度差包裹到 [-pi, pi]。
func wrapPi(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a < -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// Constraints 返回规划器使用的运动学约束（只读使用）。
func (p *Planner) Constraints() kinematics.Constraints { return p.cons }

// DT 返回仿真步长。
func (p *Planner) DT() float64 { return p.cfg.DT }

// MaxIters 返回单次局部推进的最大迭代数。
func (p *Planner) MaxIters() int { return p.cfg.MaxIters }

// Config 返回局部规划配置（值拷贝）。
func (p *Planner) Config() Config { return p.cfg }
