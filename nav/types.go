package nav

import (
	"fmt"
	"time"
)

// Pose 是机器人在世界坐标系下的二维位姿：x,y（米）与 theta（弧度）。
type Pose struct {
	X     float64
	Y     float64
	Theta float64
}

// Cell 是栅格地图中的整数坐标。
type Cell struct {
	Row int
	Col int
}

// String 以 (r,c) 形式表示栅格，便于日志输出。
func (c Cell) String() string {
	return fmt.Sprintf("(r=%d,c=%d)", c.Row, c.Col)
}

// ObstacleKind 区分静态与动态障碍物。
type ObstacleKind int

const (
	// ObstacleStatic 静态障碍物（地图固有）。
	ObstacleStatic ObstacleKind = iota
	// ObstacleDynamic 动态障碍物（传感器观测更新）。
	ObstacleDynamic
)

// Obstacle 表示一个轴对齐矩形障碍物（含膨胀半径）。
type Obstacle struct {
	ID    string
	Kind  ObstacleKind
	X     float64 // 中心 x（米）
	Y     float64 // 中心 y（米）
	HalfX float64 // x 方向半宽（米）
	HalfY float64 // y 方向半宽（米）
	// Inflation 额外膨胀量（米），用于保证安全间距。
	Inflation float64
	// Seq 是观测序号；更大的序号更新，陈旧序号将被丢弃。
	Seq int64
	// Removed 为 true 表示该动态障碍物被移除（仅对动态障碍有效）。
	Removed bool
}

// GridConfig 描述栅格地图的几何参数。
type GridConfig struct {
	Rows       int
	Cols       int
	Resolution float64 // 每格边长（米）
	OriginX    float64 // 栅格 (0,0) 中心对应的世界 x
	OriginY    float64 // 栅格 (0,0) 中心对应的世界 y
}

// Limits 描述机器人运动学与速度约束。
type Limits struct {
	MaxLinearVel  float64 // 最大线速度（米/秒），>0
	MinLinearVel  float64 // 最小（含倒车，<=0）线速度（米/秒）
	MaxAngularVel float64 // 最大角速度（弧度/秒），>0
	MaxCurvature  float64 // 最大曲率（1/米），>=0；0 表示不限制
	MaxAccel      float64 // 最大线加速度（米/秒^2），>0
	MaxSteerRate  float64 // 最大转向角速度（弧度/秒），>0
	MaxStepTheta  float64 // 相邻轨迹点航向角变化上限（弧度），>0
}

// Config 是规划器的总配置。
type Config struct {
	Grid          GridConfig
	Limits        Limits
	DT            time.Duration // 轨迹离散步长
	PlanTimeout   time.Duration // 单次规划时限
	GoalTolerance float64       // 到达目标的距离容差（米）
	Inflation     float64       // 全局障碍膨胀半径（米）
	// LocalLookahead 是局部避障检查的前瞻距离（米）。
	LocalLookahead float64
}

// PathNode 是全局路径上的一个栅格节点。
type PathNode struct {
	C Cell
	// X,Y 是该节点对应的世界坐标。
	X float64
	Y float64
}

// Path 是全局搜索得到的栅格路径。
type Path struct {
	Nodes []PathNode
}

// TrajPoint 是轨迹上的一个采样点。
type TrajPoint struct {
	Pose       Pose
	Time       time.Duration // 相对轨迹起点的时间
	LinearVel  float64
	AngularVel float64
}

// TrajStatus 标记轨迹的有效性状态。
type TrajStatus int

const (
	// TrajValid 有效轨迹，可下发执行。
	TrajValid TrajStatus = iota
	// TrajConstraintViolation 轨迹违反运动学/速度约束。
	TrajConstraintViolation
	// TrajTimeout 轨迹生成超出规划时限。
	TrajTimeout
	// TrajBlocked 局部避障判定前方被堵死。
	TrajBlocked
)

// Trajectory 是带有效性标记的离散轨迹。
type Trajectory struct {
	Points []TrajPoint
	Status TrajStatus
	// Reason 在 Status 非 Valid 时给出人类可解释原因。
	Reason string
}

// Valid 报告轨迹是否可安全下发执行。
func (t *Trajectory) Valid() bool { return t != nil && t.Status == TrajValid && len(t.Points) > 0 }

// PlanResult 汇总一次规划的输入、输出与判定依据。
type PlanResult struct {
	Start Pose
	Goal  Pose
	// Seq 是本次规划所基于的世界（地图+障碍）版本号。
	Seq       int64
	Path      *Path
	Traj      *Trajectory
	Replanned bool
	// ReplanReason 在发生重规划时说明触发原因。
	ReplanReason string
	Elapsed      time.Duration
	Err          *PlanningError
}

// Succeeded 报告本次规划是否产出可执行轨迹。
func (r *PlanResult) Succeeded() bool {
	return r != nil && r.Err == nil && r.Traj != nil && r.Traj.Valid()
}

// Validate 校验运动学约束配置的内在一致性。
func (l Limits) Validate() error {
	switch {
	case l.MaxLinearVel <= 0:
		return NewError(FailConstraint, "MaxLinearVel must be > 0, got %v", l.MaxLinearVel)
	case l.MinLinearVel > 0:
		return NewError(FailConstraint, "MinLinearVel must be <= 0, got %v", l.MinLinearVel)
	case l.MaxAngularVel <= 0:
		return NewError(FailConstraint, "MaxAngularVel must be > 0, got %v", l.MaxAngularVel)
	case l.MaxAccel <= 0:
		return NewError(FailConstraint, "MaxAccel must be > 0, got %v", l.MaxAccel)
	case l.MaxSteerRate <= 0:
		return NewError(FailConstraint, "MaxSteerRate must be > 0, got %v", l.MaxSteerRate)
	case l.MaxStepTheta <= 0:
		return NewError(FailConstraint, "MaxStepTheta must be > 0, got %v", l.MaxStepTheta)
	case l.MaxCurvature < 0:
		return NewError(FailConstraint, "MaxCurvature must be >= 0, got %v", l.MaxCurvature)
	}
	return nil
}

// Validate 校验规划总配置。
func (c Config) Validate() error {
	if c.Grid.Rows <= 0 || c.Grid.Cols <= 0 || c.Grid.Resolution <= 0 {
		return NewError(FailMapMissing,
			"invalid grid: rows=%d cols=%d res=%v", c.Grid.Rows, c.Grid.Cols, c.Grid.Resolution)
	}
	if err := c.Limits.Validate(); err != nil {
		return err
	}
	switch {
	case c.DT <= 0:
		return NewError(FailConstraint, "DT must be > 0, got %v", c.DT)
	case c.PlanTimeout <= 0:
		return NewError(FailTimeout, "PlanTimeout must be > 0, got %v", c.PlanTimeout)
	case c.GoalTolerance < 0:
		return NewError(FailGoalInvalid, "GoalTolerance must be >= 0, got %v", c.GoalTolerance)
	case c.Inflation < 0:
		return NewError(FailObstacleInvalid, "Inflation must be >= 0, got %v", c.Inflation)
	case c.LocalLookahead < 0:
		return NewError(FailConstraint, "LocalLookahead must be >= 0, got %v", c.LocalLookahead)
	}
	return nil
}
