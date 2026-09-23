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
