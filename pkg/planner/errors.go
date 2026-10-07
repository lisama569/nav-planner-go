// Package planner 实现确定性的全局栅格路径搜索（A*）与重规划。
package planner

import "errors"

// 规划结果的可区分错误原因。
var (
	ErrMapMissing       = errors.New("planner: map is missing")
	ErrStartOutOfBounds = errors.New("planner: start is out of bounds")
	ErrStartOccupied    = errors.New("planner: start cell is occupied")
	ErrGoalOutOfBounds  = errors.New("planner: goal is out of bounds")
	ErrGoalOccupied     = errors.New("planner: goal cell is occupied")
	ErrUnreachable      = errors.New("planner: goal is unreachable")
	ErrPlanLimit        = errors.New("planner: expansion budget exhausted")
	ErrPlanTimeout      = errors.New("planner: wall-clock deadline exceeded")
)
