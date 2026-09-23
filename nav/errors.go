package nav

import "fmt"

// FailCode 是对规划失败原因的可区分分类。
type FailCode string

const (
	// FailNone 表示无失败。
	FailNone FailCode = ""
	// FailMapMissing 地图缺失。
	FailMapMissing FailCode = "map_missing"
	// FailStartInvalid 起点非法（越界或落入障碍）。
	FailStartInvalid FailCode = "start_invalid"
	// FailGoalInvalid 目标非法（越界或落入障碍）。
	FailGoalInvalid FailCode = "goal_invalid"
	// FailUnreachable 目标在当前可达区域内不可达。
	FailUnreachable FailCode = "goal_unreachable"
	// FailObstacleInvalid 障碍物数据非法。
	FailObstacleInvalid FailCode = "obstacle_invalid"
	// FailNoPath 当前条件下无路径（例如超时前未搜到）。
	FailNoPath FailCode = "no_path"
	// FailTimeout 规划超出时限。
	FailTimeout FailCode = "planning_timeout"
	// FailConstraint 轨迹违反运动学/速度约束。
	FailConstraint FailCode = "constraint_violation"
	// FailStaleUpdate 更新序号陈旧被丢弃。
	FailStaleUpdate FailCode = "stale_update"
	// FailNotInitialized 导航器未初始化或未设置地图。
	FailNotInitialized FailCode = "not_initialized"
)

// PlanningError 携带可区分的失败分类与解释信息。
type PlanningError struct {
	Code FailCode
	Msg  string
}

func (e *PlanningError) Error() string {
	if e == nil {
		return ""
	}
	return fmt.Sprintf("nav: %s: %s", e.Code, e.Msg)
}

// NewError 构造一个带分类码的规划错误。
func NewError(code FailCode, format string, args ...any) *PlanningError {
	return &PlanningError{Code: code, Msg: fmt.Sprintf(format, args...)}
}

// AsPlanningError 从任意错误中提取 *PlanningError。
func AsPlanningError(err error) (*PlanningError, bool) {
	if err == nil {
		return nil, false
	}
	if pe, ok := err.(*PlanningError); ok {
		return pe, true
	}
	return nil, false
}
