package replay

import (
	"testing"

	"github.com/lisama569/nav-planner-go/pkg/grid"
	"github.com/lisama569/nav-planner-go/pkg/kinematics"
	"github.com/lisama569/nav-planner-go/pkg/local"
	"github.com/lisama569/nav-planner-go/pkg/nav"
	"github.com/lisama569/nav-planner-go/pkg/planner"
)

func twoGapMap() *grid.Grid {
	g := grid.New(15, 10)
	for y := 0; y < 10; y++ {
		if y == 4 || y == 0 {
			continue
		}
		g.Set(grid.Cell{X: 7, Y: y}, true)
	}
	return g
}

func newNav() *nav.Navigator {
	lp := local.New(local.Config{DT: 0.25, Horizon: 6, VSamples: 3, OmegaSamples: 7,
		GoalTolerance: 0.4, ObstacleMargin: 0.25, MaxIters: 80}, kinematics.DefaultConstraints())
	return nav.New(planner.New(0), lp, nil)
}

func TestReplayIdentical(t *testing.T) {
	base := twoGapMap()
	start := kinematics.Pose{X: 1, Y: 1}
	goal := grid.Cell{X: 13, Y: 8}

	rec := NewRecorder(newNav())
	rec.Apply(Event{Tick: 1, Kind: EventMap, Map: base})
	r1 := rec.Apply(Event{Tick: 100, Kind: EventRequest, Start: start, Goal: goal})
	t.Logf("[replay] 首次请求 status=%s 路径长=%d", r1.Status, len(r1.Path))
	rec.Apply(Event{Tick: 2, Kind: EventObstacle, Obstacle: local.Obstacle{ID: "m", X: 7, Y: 4, Radius: 0.7, Seq: 2}})
	r2 := rec.Apply(Event{Tick: 101, Kind: EventRequest, Start: start, Goal: goal})
	t.Logf("[replay] 遇障请求 status=%s replans=%d", r2.Status, r2.Replans)
	rec.Apply(Event{Tick: 3, Kind: EventObstacle, Obstacle: local.Obstacle{ID: "s", X: 7, Y: 0, Radius: 0.7, Seq: 3}})
	r3 := rec.Apply(Event{Tick: 102, Kind: EventRequest, Start: start, Goal: goal})
	t.Logf("[replay] 封堵请求 status=%s reason=%v", r3.Status, r3.Reason)

	ok, err := Replay(rec.Log(), newNav())
	t.Logf("[replay] 重放 %d 条目，完全一致=%v err=%v", len(rec.Log().Entries), ok, err)
	if !ok || err != nil {
		t.Fatalf("replay must reproduce identical results: %v", err)
	}
}

func TestReplayDetectsDivergence(t *testing.T) {
	base := twoGapMap()
	rec := NewRecorder(newNav())
	rec.Apply(Event{Tick: 1, Kind: EventMap, Map: base})
	rec.Apply(Event{Tick: 100, Kind: EventRequest, Start: kinematics.Pose{X: 1, Y: 1}, Goal: grid.Cell{X: 13, Y: 8}})

	// 用一个不同地图（堵死近缺口）的全新编排器重放，应报告差异
	different := grid.New(15, 10)
	rec2 := NewRecorder(newNav())
	rec2.Apply(Event{Tick: 1, Kind: EventMap, Map: different})
	ok, err := Replay(rec.Log(), rec2.nav)
	t.Logf("[replay-diverge] 一致=%v err=%v（期望false+差异描述）", ok, err)
	if ok || err == nil {
		t.Fatal("replay must detect divergence when maps differ")
	}
}

// newNavWithLimit 允许重放测试注入带虚拟时钟时限的规划器。
func newNavWithLimit(maxExp int, deadlineUs int64) *nav.Navigator {
	lp := local.New(local.Config{DT: 0.25, Horizon: 6, VSamples: 3, OmegaSamples: 7,
		GoalTolerance: 0.4, ObstacleMargin: 0.25, MaxIters: 80}, kinematics.DefaultConstraints())
	gp := planner.New(0)
	if deadlineUs > 0 {
		gp = planner.NewWithLimit(planner.Limit{
			MaxExpansions: maxExp, DeadlineMicros: deadlineUs,
			Clock: planner.NewManualClock(1000),
		})
	}
	return nav.New(gp, lp, nil)
}

// TestReplayOneShotAndObstacles 验证新增“一次性入口 + 动态障碍计入”下录制/重放
// 仍然逐位一致；且同一场景的一次性 Plan 与逐步 ReplanningStep 录制结论相同。
func TestReplayOneShotAndObstacles(t *testing.T) {
	base := twoGapMap()
	start := kinematics.Pose{X: 1, Y: 1}
	goal := grid.Cell{X: 13, Y: 8}

	build := func(oneShot bool) *Log {
		rec := NewRecorder(newNav())
		rec.Apply(Event{Tick: 1, Kind: EventMap, Map: base})
		rec.Apply(Event{Tick: 2, Kind: EventObstacle, Obstacle: local.Obstacle{ID: "m", X: 7, Y: 4, Radius: 0.7, Seq: 2}})
		r := rec.Apply(Event{Tick: 100, Kind: EventRequest, Start: start, Goal: goal, OneShot: oneShot})
		t.Logf("[replay-oneshot=%v] 遇障请求 status=%s replans=%d 路径长=%d",
			oneShot, r.Status, r.Replans, len(r.Path))
		return rec.Log()
	}

	logOne := build(true)
	logStep := build(false)
	// 两个入口对同一世界的结论与路径应完全一致
	if e1, e2 := logOne.Entries[2].Result, logStep.Entries[2].Result; e1.Status != e2.Status ||
		len(e1.Path) != len(e2.Path) {
		t.Fatalf("入口录制结论不一致: oneShot=%+v step=%+v", e1.Status, e2.Status)
	}

	for _, l := range []*Log{logOne, logStep} {
		ok, err := Replay(l, newNav())
		t.Logf("[replay-oneshot] 重放 %d 条目，完全一致=%v err=%v", len(l.Entries), ok, err)
		if !ok || err != nil {
			t.Fatalf("重放必须逐位一致: %v", err)
		}
	}
}

// TestReplayTimeoutDeterministic 验证“超时”作为可区分结论可被录制/重放：
// 虚拟时钟驱动下，重放必须复现同一个 TIMEOUT，且不会退化为成功或 PLAN_LIMIT。
func TestReplayTimeoutDeterministic(t *testing.T) {
	big := grid.New(200, 200)
	start := kinematics.Pose{X: 1, Y: 1}
	goal := grid.Cell{X: 198, Y: 198}

	rec := NewRecorder(newNavWithLimit(1_000_000, 2000))
	rec.Apply(Event{Tick: 1, Kind: EventMap, Map: big})
	r := rec.Apply(Event{Tick: 100, Kind: EventRequest, Start: start, Goal: goal, OneShot: true})
	t.Logf("[replay-timeout] 录制 status=%s reason=%v", r.Status, r.Reason)
	if r.Status != nav.StatusTimeout {
		t.Fatalf("want TIMEOUT got %s", r.Status)
	}
	ok, err := Replay(rec.Log(), newNavWithLimit(1_000_000, 2000))
	t.Logf("[replay-timeout] 重放一致=%v err=%v", ok, err)
	if !ok || err != nil {
		t.Fatalf("超时结论重放必须复现同一个 TIMEOUT: %v", err)
	}
}
