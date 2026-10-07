package nav

import "errors"

// 编排层错误；输入类问题的具体原因直接透传 planner/kinematics 包的哨兵错误。
var (
	// ErrMapMissing 表示尚未装载地图就请求规划。
	ErrMapMissing = errors.New("nav: map is missing")
	// ErrStaleUpdate 表示到达的更新序号落后于已应用序号，按规则丢弃。
	ErrStaleUpdate = errors.New("nav: stale update dropped")
	// ErrInvalidUpdate 表示更新载荷非法（如 nil 地图）。
	ErrInvalidUpdate = errors.New("nav: invalid update payload")
	// ErrConstraintViolation 表示生成的轨迹违反运动学/速度约束，不得下发。
	ErrConstraintViolation = errors.New("nav: trajectory violates kinematic constraints")
	// ErrNoPlanner 表示编排器缺少必要的规划器组件。
	ErrNoPlanner = errors.New("nav: planner not configured")
	// ErrPlanTimeout 表示触发可选的墙钟兜底时限；结论分类为 StatusPlanLimit，
	// 与确定性预算耗尽（planner.ErrPlanLimit）同属“超时/未完成”类，
	// 可用 errors.Is(reason, ErrPlanTimeout) 区分触发来源。不下发任何路径/轨迹。
	ErrPlanTimeout = errors.New("nav: planning time limit exceeded")
)
