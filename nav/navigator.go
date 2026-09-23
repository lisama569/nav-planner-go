package nav

import (
	"sync"
	"time"
)

// Navigator 是并发安全的导航编排器。
//
// 并发模型：
//   - 一把 sync.Mutex 保护全部可变状态（基础地图引用、动态障碍表、
//     版本号 Seq、最近一次结果）；
//   - 规划时先在锁内完成“克隆地图 + 叠加动态障碍”得到不可变工作副本，
//     随后释放锁执行 A* 与轨迹生成，避免长耗时搜索阻塞传感器更新；
//   - 所有结果在落库前再次取锁，并核对世界版本，防止旧结果覆盖新状态。
//
// 序号规则（陈旧/乱序更新）：
//   - 动态障碍按 (ID, Seq) 处理；新消息 Seq 必须 >= 该 ID 已见 Seq，
//     否则视为陈旧消息丢弃（accepted=false, FailStaleUpdate 原因）；
//   - Seq 相等的重复消息幂等丢弃；
//   - 世界版本 Seq 在每次被接受的更新后 +1，结果通过 PlanResult.Seq
//     标明它基于哪个版本。
type Navigator struct {
	cfg Config
	clk Clock

	mu      sync.Mutex
	base    *GridMap
	dyn     map[string]Obstacle
	lastSeq map[string]int64
	seq     int64
	last    *PlanResult
	updated time.Time
}

// NewNavigator 创建导航器并校验配置。时钟为 nil 时使用墙钟。
func NewNavigator(cfg Config, clk Clock) (*Navigator, error) {
	if err := cfg.Validate(); err != nil {
		return nil, err
	}
	if clk == nil {
		clk = WallClock{}
	}
	return &Navigator{
		cfg:     cfg,
		clk:     clk,
		dyn:     map[string]Obstacle{},
		lastSeq: map[string]int64{},
	}, nil
}

// SetMap 安装（或替换）基础地图，并将静态障碍物栅格化进地图。
// 替换地图不会清除动态障碍表；后续规划会把动态障碍叠加到新地图上。
func (n *Navigator) SetMap(m *GridMap, statics []Obstacle) error {
	if m == nil {
		return NewError(FailMapMissing, "SetMap received nil map")
	}
	work := m.Clone()
	for _, o := range statics {
		o.Kind = ObstacleStatic
		if err := work.StampObstacle(o); err != nil {
			return err
		}
	}
	n.mu.Lock()
	n.base = work
	n.updated = n.clk.Now()
	n.mu.Unlock()
	return nil
}

// ApplyObstacle 处理一个动态障碍更新（含移除）。
// 返回 (accepted, reason)：陈旧/重复/非法消息 accepted=false。
func (n *Navigator) ApplyObstacle(o Obstacle) (bool, string) {
	if o.ID == "" {
		return false, NewError(FailObstacleInvalid, "dynamic obstacle id is empty").Error()
	}
	n.mu.Lock()
	defer n.mu.Unlock()

	if prev, ok := n.lastSeq[o.ID]; ok {
		switch {
		case o.Seq < prev:
			return false, NewError(FailStaleUpdate,
				"obstacle %q update seq=%d is older than last seq=%d (dropped)",
				o.ID, o.Seq, prev).Error()
		case o.Seq == prev:
			return false, NewError(FailStaleUpdate,
				"obstacle %q update seq=%d is a duplicate (ignored)", o.ID, o.Seq).Error()
		}
	}

	// 几何校验：即使是移除消息也不强制几何，但非移除消息必须合法。
	if !o.Removed {
		tmp := NewGridMap(n.cfg.Grid)
		if err := tmp.StampObstacle(o); err != nil {
			return false, err.Error()
		}
	}

	o.Kind = ObstacleDynamic
	n.lastSeq[o.ID] = o.Seq
	if o.Removed {
		delete(n.dyn, o.ID)
	} else {
		n.dyn[o.ID] = o
	}
	n.seq++
	n.updated = n.clk.Now()
	return true, ""
}

// Plan 执行一次从 start 到 goal 的规划（含动态重规划）。
//
// 结果永不为 nil；失败时 Err 携带可区分的 FailCode。
// 当检测到动态障碍导致当前轨迹受阻时，Replanned=true 并记录
// ReplanReason。目标不可达/起终点非法/地图缺失等在搜索阶段即被拒绝。
func (n *Navigator) Plan(start, goal Pose) *PlanResult {
	t0 := n.clk.Now()
	deadline := t0.Add(n.cfg.PlanTimeout)
	res := &PlanResult{Start: start, Goal: goal}

	// 阶段 1：锁内生成不可变世界副本。
	n.mu.Lock()
	if n.base == nil {
		n.mu.Unlock()
		res.Err = NewError(FailNotInitialized, "Plan called before SetMap")
		res.Elapsed = n.clk.Now().Sub(t0)
		return res
	}
	world := n.base.Clone()
	dynCopy := make(map[string]Obstacle, len(n.dyn))
	for id, o := range n.dyn {
		dynCopy[id] = o
	}
	var lastTraj *Trajectory
	prevGoal := poseZero
	hadResult := n.last != nil && n.last.Succeeded()
	if hadResult {
		prevGoal = n.last.Goal
		lastTraj = n.last.Traj
	}
	worldSeq := n.seq
	n.mu.Unlock()

	for _, o := range dynCopy {
		if err := world.StampObstacle(o); err != nil {
			res.Err = err.(*PlanningError)
			res.Elapsed = n.clk.Now().Sub(t0)
			return res
		}
	}
	res.Seq = worldSeq

	// 重规划触发依据：导航进行中（已有成功轨迹、目标未变），
	// 且“正在执行的旧轨迹”在叠加最新障碍后前瞻走廊被堵。
	replanning := false
	blockReason := ""
	if hadResult && sameXYGoal(prevGoal, goal, n.cfg.GoalTolerance) {
		lv := NewLocalAvoider(n.cfg).Check(world, lastTraj, start, dynCopy)
		replanning = lv.Blocked
		blockReason = lv.Reason
	}

	// 阶段 2：锁外做耗时搜索。
	gp := NewGlobalPlanner(n.cfg)
	path, err := gp.Search(world, start, goal, &deadline, n.clk)
	if err != nil {
		pe, _ := AsPlanningError(err)
		res.Err = pe
		res.Elapsed = n.clk.Now().Sub(t0)
		res.Replanned = replanning
		if replanning {
			res.ReplanReason = "trigger: " + blockReason + "; replanning search failed: " + pe.Msg
		}
		n.commit(res, t0)
		return res
	}
	res.Path = path

	// 阶段 3：轨迹生成（同一时限预算）。
	tb := NewTrajectoryBuilder(n.cfg)
	traj := tb.Build(start, goal, path, &deadline, n.clk)
	if traj.Status != TrajValid {
		code := FailConstraint
		if traj.Status == TrajTimeout {
			code = FailTimeout
		}
		res.Err = NewError(code, "trajectory rejected: %s", traj.Reason)
		res.Traj = traj
		res.Elapsed = n.clk.Now().Sub(t0)
		res.Replanned = replanning
		n.commit(res, t0)
		return res
	}

	// 阶段 4：新轨迹构建在与搜索相同的世界副本上，必然无碰撞；
	// 再做一次自检以提供可解释证据（理论上恒为无冲突）。
	lv := NewLocalAvoider(n.cfg).Check(world, traj, start, dynCopy)
	res.Traj = traj
	res.Replanned = replanning
	if replanning {
		res.ReplanReason = "trigger: " + blockReason +
			"; replanned on world seq=" + itoa64(worldSeq) +
			"; new corridor check: " + lv.Reason
	}
	n.commit(res, t0)
	return res
}

// commit 在锁内写入最新结果，并核对世界版本未在规划期间回退。
func (n *Navigator) commit(res *PlanResult, t0 time.Time) {
	n.mu.Lock()
	res.Elapsed = n.clk.Now().Sub(t0)
	n.last = cloneResult(res)
	n.updated = n.clk.Now()
	n.mu.Unlock()
}

// State 返回当前状态的只读一致性快照。
func (n *Navigator) State() Snapshot {
	n.mu.Lock()
	defer n.mu.Unlock()
	obsCopy := make(map[string]Obstacle, len(n.dyn))
	for id, o := range n.dyn {
		obsCopy[id] = o
	}
	snap := Snapshot{
		HasMap:    n.base != nil,
		Seq:       n.seq,
		DynCount:  len(n.dyn),
		Obstacles: obsCopy,
		UpdatedAt: n.updated,
	}
	if n.last != nil {
		snap.LastResult = cloneResult(n.last)
	}
	return snap
}

// cloneResult 深拷贝结果，避免调用方持有的切片与内部状态别名共享。
func cloneResult(r *PlanResult) *PlanResult {
	if r == nil {
		return nil
	}
	cp := *r
	if r.Path != nil {
		nodes := make([]PathNode, len(r.Path.Nodes))
		copy(nodes, r.Path.Nodes)
		cp.Path = &Path{Nodes: nodes}
	}
	if r.Traj != nil {
		pts := make([]TrajPoint, len(r.Traj.Points))
		copy(pts, r.Traj.Points)
		cp.Traj = &Trajectory{Points: pts, Status: r.Traj.Status, Reason: r.Traj.Reason}
	}
	return &cp
}

func sameXYGoal(a, b Pose, tol float64) bool {
	dx, dy := a.X-b.X, a.Y-b.Y
	return dx*dx+dy*dy <= tol*tol
}

var poseZero Pose

func itoa64(v int64) string {
	if v == 0 {
		return "0"
	}
	neg := v < 0
	if neg {
		v = -v
	}
	var b [20]byte
	i := len(b)
	for v > 0 {
		i--
		b[i] = byte('0' + v%10)
		v /= 10
	}
	if neg {
		i--
		b[i] = '-'
	}
	return string(b[i:])
}

// Snapshot 是导航器状态的只读快照。
type Snapshot struct {
	HasMap     bool
	Seq        int64
	DynCount   int
	Obstacles  map[string]Obstacle // 存活动态障碍的副本（按 ID）
	LastResult *PlanResult
	UpdatedAt  time.Time
}
