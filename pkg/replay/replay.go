// Package replay 提供确定性场景录制与重放。
//
// 使用方式：按逻辑时间（Tick）依次向 Recorder 投递地图/障碍物/请求事件，
// Recorder 同步调用 Navigator 并记录结论；之后用一个全新 Navigator 重放整条
// 日志，逐条比较状态、错误原因、路径、重规划次数与轨迹点位姿，
// 任意不一致即报告第一个差异点。
package replay

import (
	"fmt"

	"github.com/lisama569/nav-planner-go/pkg/grid"
	"github.com/lisama569/nav-planner-go/pkg/kinematics"
	"github.com/lisama569/nav-planner-go/pkg/local"
	"github.com/lisama569/nav-planner-go/pkg/nav"
)

// EventKind 标识录制事件类型。
type EventKind int

const (
	EventMap EventKind = iota + 1
	EventObstacle
	EventRequest
)

func (k EventKind) String() string {
	switch k {
	case EventMap:
		return "MAP"
	case EventObstacle:
		return "OBSTACLE"
	case EventRequest:
		return "REQUEST"
	default:
		return "UNKNOWN"
	}
}

// Event 是按逻辑时间排序的一条输入事件。
type Event struct {
	Tick     int64
	Kind     EventKind
	Map      *grid.Grid
	Obstacle local.Obstacle
	Removed  bool
	Start    kinematics.Pose
	Goal     grid.Cell
}

// Entry 是事件与其对应结论的一条录制记录。
type Entry struct {
	Event     Event
	Result    nav.Result
	UpdateErr error // 仅 MAP/OBSTACLE 事件使用
}

// Log 是一次完整场景运行的确定性记录。
type Log struct {
	Entries []Entry
}

// Recorder 在 Navigator 之外录制事件与结论。
type Recorder struct {
	nav *nav.Navigator
	log Log
}

// NewRecorder 创建录制器。
func NewRecorder(n *nav.Navigator) *Recorder { return &Recorder{nav: n} }

// Apply 应用一条事件并录制结论；事件必须按 Tick 递增投递。
func (r *Recorder) Apply(e Event) nav.Result {
	var res nav.Result
	var upErr error
	switch e.Kind {
	case EventMap:
		upErr = r.nav.ApplyMap(nav.MapUpdate{Seq: e.Tick, Grid: e.Map})
		res = nav.Result{ReqSeq: e.Tick, Tick: e.Tick}
		if upErr == nil {
			res.Status = nav.StatusPlanned // 借用“成功”语义表示更新被接受
		} else {
			res.Status = nav.StatusRejected
			res.Reason = upErr
		}
	case EventObstacle:
		upErr = r.nav.ApplyObstacle(nav.ObstacleUpdate{Seq: e.Tick, Obstacle: e.Obstacle, Removed: e.Removed})
		res = nav.Result{ReqSeq: e.Tick, Tick: e.Tick}
		if upErr == nil {
			res.Status = nav.StatusPlanned
		} else {
			res.Status = nav.StatusRejected
			res.Reason = upErr
		}
	case EventRequest:
		res = r.nav.ReplanningStep(nav.Request{
			Seq: e.Tick, Tick: e.Tick, Start: e.Start, Goal: e.Goal,
		})
	}
	r.log.Entries = append(r.log.Entries, Entry{Event: e, Result: res, UpdateErr: upErr})
	return res
}

// Log 返回已录制的日志。
func (r *Recorder) Log() *Log { return &r.log }

// Replay 用全新 Navigator 重放日志并逐条比对结论是否一致。
// 一致返回 (true,nil)；不一致返回 (false, 描述首个差异的 error)。
func Replay(l *Log, fresh *nav.Navigator) (bool, error) {
	rr := NewRecorder(fresh)
	for i, want := range l.Entries {
		got := rr.Apply(want.Event)
		if err := compareResult(want.Result, got); err != nil {
			return false, fmt.Errorf("replay mismatch at entry %d (tick=%d, %s): %w",
				i, want.Event.Tick, want.Event.Kind, err)
		}
	}
	return true, nil
}

// compareResult 比较两条结论的全部对外可观测字段。
func compareResult(want, got nav.Result) error {
	if want.Status != got.Status {
		return fmt.Errorf("status want=%s got=%s", want.Status, got.Status)
	}
	if !sameReason(want.Reason, got.Reason) {
		return fmt.Errorf("reason want=%v got=%v", want.Reason, got.Reason)
	}
	if want.Replans != got.Replans {
		return fmt.Errorf("replans want=%d got=%d", want.Replans, got.Replans)
	}
	if len(want.Path) != len(got.Path) {
		return fmt.Errorf("path len want=%d got=%d", len(want.Path), len(got.Path))
	}
	for i := range want.Path {
		if want.Path[i] != got.Path[i] {
			return fmt.Errorf("path[%d] want=%v got=%v", i, want.Path[i], got.Path[i])
		}
	}
	return compareTrajectory(want.Traj, got.Traj)
}

func sameReason(a, b error) bool {
	if a == nil || b == nil {
		return a == nil && b == nil
	}
	return a.Error() == b.Error()
}

// compareTrajectory 逐点精确比较；同一确定性流水线下浮点运算顺序一致，
// 重放结果应逐位相同，因此这里不使用近似比较（近似会掩盖真实的不确定性）。
func compareTrajectory(a, b *kinematics.Trajectory) error {
	na, nb := 0, 0
	if a != nil {
		na = len(a.Points)
	}
	if b != nil {
		nb = len(b.Points)
	}
	if na != nb {
		return fmt.Errorf("trajectory len want=%d got=%d", na, nb)
	}
	if na == 0 {
		return nil
	}
	for i := range a.Points {
		if a.Points[i] != b.Points[i] {
			return fmt.Errorf("trajectory[%d] want=%+v got=%+v", i, a.Points[i], b.Points[i])
		}
	}
	return nil
}
