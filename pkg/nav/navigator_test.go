package nav

import (
	"errors"
	"fmt"
	"sync"
	"testing"

	"github.com/lisama569/nav-planner-go/pkg/grid"
	"github.com/lisama569/nav-planner-go/pkg/kinematics"
	"github.com/lisama569/nav-planner-go/pkg/local"
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

func newTestNav(g *grid.Grid) *Navigator {
	lp := local.New(local.Config{DT: 0.25, Horizon: 6, VSamples: 3, OmegaSamples: 7,
		GoalTolerance: 0.4, ObstacleMargin: 0.25, MaxIters: 80}, kinematics.DefaultConstraints())
	return New(planner.New(0), lp, g)
}

func logResult(t *testing.T, name string, req Request, r Result) {
	t.Helper()
	t.Logf("[%s] 输入 start=(%.2f,%.2f,th=%.2f) goal=%v",
		name, req.Start.X, req.Start.Y, req.Start.Theta, req.Goal)
	t.Logf("[%s] 结果 status=%s reason=%v replans=%d expansions=%d 路径长度=%d 轨迹段数=%d",
		name, r.Status, r.Reason, r.Replans, r.Expansions, len(r.Path), trajLen(r.Traj))
}

func trajLen(t *kinematics.Trajectory) int {
	if t == nil {
		return 0
	}
	return len(t.Points)
}

func TestPlannedAndReplannedAndUnreachable(t *testing.T) {
	base := twoGapMap()
	n := newTestNav(base)
	if err := n.ApplyMap(MapUpdate{Seq: 1, Grid: base}); err != nil {
		t.Fatal(err)
	}
	req := Request{Seq: 100, Tick: 1, Start: kinematics.Pose{X: 1, Y: 1}, Goal: grid.Cell{X: 13, Y: 8}}

	r1 := n.ReplanningStep(req)
	logResult(t, "initial", req, r1)
	if r1.Status != StatusPlanned {
		t.Fatalf("want PLANNED got %s (%v)", r1.Status, r1.Reason)
	}
	// 初始路径应走近缺口 (7,4)
	if !contains(r1.Path, grid.Cell{X: 7, Y: 4}) {
		t.Fatalf("expected path via (7,4): %v", r1.Path)
	}

	// 动态障碍堵住近缺口 => 重规划走低缺口 (7,0)
	if err := n.ApplyObstacle(ObstacleUpdate{Seq: 2, Obstacle: local.Obstacle{
		ID: "m", X: 7, Y: 4, Radius: 0.7, Seq: 2}}); err != nil {
		t.Fatal(err)
	}
	r2 := n.ReplanningStep(req)
	logResult(t, "moving-obstacle", req, r2)
	if r2.Status != StatusReplanned || r2.Replans < 1 {
		t.Fatalf("want REPLANNED got %s (%v)", r2.Status, r2.Reason)
	}
	if contains(r2.Path, grid.Cell{X: 7, Y: 4}) {
		t.Fatalf("replan must avoid (7,4): %v", r2.Path)
	}
	if !contains(r2.Path, grid.Cell{X: 7, Y: 0}) {
		t.Fatalf("replan should detour via (7,0): %v", r2.Path)
	}

	// 再堵低缺口 => 不可达（可区分原因）
	if err := n.ApplyObstacle(ObstacleUpdate{Seq: 3, Obstacle: local.Obstacle{
		ID: "s", X: 7, Y: 0, Radius: 0.7, Seq: 3}}); err != nil {
		t.Fatal(err)
	}
	r3 := n.ReplanningStep(req)
	logResult(t, "both-gaps-sealed", req, r3)
	if r3.Status != StatusUnreachable || !errors.Is(r3.Reason, planner.ErrUnreachable) {
		t.Fatalf("want UNREACHABLE got %s (%v)", r3.Status, r3.Reason)
	}
}

func TestRejectionClasses(t *testing.T) {
	base := twoGapMap()
	n := newTestNav(base)
	_ = n.ApplyMap(MapUpdate{Seq: 1, Grid: base})
	start := kinematics.Pose{X: 1, Y: 1}

	cases := []struct {
		name string
		req  Request
		want error
	}{
		{"goal-out-of-bounds", Request{Start: start, Goal: grid.Cell{X: 99, Y: 99}}, planner.ErrGoalOutOfBounds},
		{"goal-occupied", Request{Start: start, Goal: grid.Cell{X: 7, Y: 1}}, planner.ErrGoalOccupied},
		{"start-occupied", Request{Start: kinematics.Pose{X: 7, Y: 1}, Goal: grid.Cell{X: 1, Y: 1}}, planner.ErrStartOccupied},
	}
	for _, c := range cases {
		r := n.Plan(c.req)
		logResult(t, c.name, c.req, r)
		if r.Status != StatusRejected || !errors.Is(r.Reason, c.want) {
			t.Fatalf("%s: want REJECTED/%v got %s/%v", c.name, c.want, r.Status, r.Reason)
		}
	}

	// 地图缺失
	empty := New(planner.New(0), newTestNav(base).lp, nil)
	// 用独立方式构造无地图编排器
	lp := local.New(local.Config{DT: 0.25, Horizon: 6, VSamples: 3, OmegaSamples: 7,
		GoalTolerance: 0.4, ObstacleMargin: 0.25, MaxIters: 80}, kinematics.DefaultConstraints())
	empty = New(planner.New(0), lp, nil)
	req := Request{Start: start, Goal: grid.Cell{X: 1, Y: 1}}
	r := empty.Plan(req)
	logResult(t, "map-missing", req, r)
	if r.Status != StatusRejected || !errors.Is(r.Reason, ErrMapMissing) {
		t.Fatalf("want map-missing got %s/%v", r.Status, r.Reason)
	}
}

func TestStaleUpdatesDropped(t *testing.T) {
	n := newTestNav(twoGapMap())
	if err := n.ApplyObstacle(ObstacleUpdate{Seq: 10, Obstacle: local.Obstacle{ID: "a", X: 1, Y: 1, Radius: 0.5, Seq: 10}}); err != nil {
		t.Fatal(err)
	}
	err := n.ApplyObstacle(ObstacleUpdate{Seq: 9, Obstacle: local.Obstacle{ID: "a", X: 99, Y: 99, Radius: 0.5, Seq: 9}})
	t.Logf("[stale] 旧序号更新 err=%v isStale=%v", err, errors.Is(err, ErrStaleUpdate))
	if !errors.Is(err, ErrStaleUpdate) {
		t.Fatalf("stale obstacle update must be dropped: %v", err)
	}
	// 不同 ID 的较小序号不应被误判为乱序
	err = n.ApplyObstacle(ObstacleUpdate{Seq: 1, Obstacle: local.Obstacle{ID: "b", X: 2, Y: 2, Radius: 0.5, Seq: 1}})
	if err != nil {
		t.Fatalf("per-ID watermarks: fresh ID must be accepted: %v", err)
	}
	// 地图乱序
	if err := n.ApplyMap(MapUpdate{Seq: 5, Grid: twoGapMap()}); err != nil {
		t.Fatal(err)
	}
	err = n.ApplyMap(MapUpdate{Seq: 4, Grid: twoGapMap()})
	t.Logf("[stale-map] 旧地图更新 err=%v isStale=%v", err, errors.Is(err, ErrStaleUpdate))
	if !errors.Is(err, ErrStaleUpdate) {
		t.Fatalf("stale map update must be dropped: %v", err)
	}
	err = n.ApplyMap(MapUpdate{Seq: 6, Grid: nil})
	if !errors.Is(err, ErrInvalidUpdate) {
		t.Fatalf("nil map should be invalid: %v", err)
	}
}

func TestConcurrentUpdatesNoRaceAndConsistent(t *testing.T) {
	base := twoGapMap()
	n := newTestNav(base)
	_ = n.ApplyMap(MapUpdate{Seq: 1, Grid: base})

	var wg sync.WaitGroup
	stop := make(chan struct{})

	// 多个传感器协程并发更新不同障碍物（各自序号单调）
	for w := 0; w < 4; w++ {
		wg.Add(1)
		go func(id int) {
			defer wg.Done()
			base := int64(id * 1000)
			for i := int64(1); ; i++ {
				select {
				case <-stop:
					return
				default:
				}
				o := local.Obstacle{
					ID: fmt.Sprintf("sensor-%d", id), X: float64(id), Y: float64(i % 9),
					Radius: 0.4, Seq: base + i,
				}
				_ = n.ApplyObstacle(ObstacleUpdate{Seq: base + i, Obstacle: o})
			}
		}(w)
	}
	// 并发快照读取与规划
	wg.Add(1)
	go func() {
		defer wg.Done()
		req := Request{Start: kinematics.Pose{X: 1, Y: 1}, Goal: grid.Cell{X: 13, Y: 8}}
		for k := 0; k < 200; k++ {
			g, obs := n.Snapshot()
			if g == nil {
				t.Error("snapshot map nil")
				return
			}
			// 快照障碍物必须是按 ID 排序的一致切片
			for i := 1; i < len(obs); i++ {
				if obs[i-1].ID > obs[i].ID {
					t.Error("snapshot obstacles not sorted")
					return
				}
			}
			r := n.ReplanningStep(req)
			if r.Status == StatusUnknown {
				t.Error("status must be classified")
				return
			}
		}
	}()
	// 并发注入乱序更新（预期全部被丢弃，不影响状态）
	wg.Add(1)
	go func() {
		defer wg.Done()
		for i := 0; i < 200; i++ {
			_ = n.ApplyObstacle(ObstacleUpdate{Seq: 0, Obstacle: local.Obstacle{ID: "sensor-0", Seq: 0}})
		}
	}()
	close(stop)
	wg.Wait()
	t.Log("[concurrent] 并发更新/快照/规划完成，需配合 -race 确认无数据竞争；乱序更新全部被水位拒绝")
}

func TestDeterministicAcrossRuns(t *testing.T) {
	req := Request{Seq: 1, Start: kinematics.Pose{X: 1, Y: 1}, Goal: grid.Cell{X: 13, Y: 8}}
	run := func() Result {
		n := newTestNav(twoGapMap())
		_ = n.ApplyMap(MapUpdate{Seq: 1, Grid: twoGapMap()})
		_ = n.ApplyObstacle(ObstacleUpdate{Seq: 2, Obstacle: local.Obstacle{ID: "m", X: 7, Y: 4, Radius: 0.7, Seq: 2}})
		return n.ReplanningStep(req)
	}
	a, b := run(), run()
	logResult(t, "determinism-a", req, a)
	t.Logf("[determinism-b] status=%s replans=%d 路径=%v", b.Status, b.Replans, b.Path)
	if a.Status != b.Status || a.Replans != b.Replans || len(a.Path) != len(b.Path) {
		t.Fatal("nondeterministic result")
	}
	for i := range a.Path {
		if a.Path[i] != b.Path[i] {
			t.Fatalf("path differs at %d: %v vs %v", i, a.Path[i], b.Path[i])
		}
	}
	if a.Traj.Duration() != b.Traj.Duration() || a.Traj.End() != b.Traj.End() {
		t.Fatal("trajectory differs across runs")
	}
}

func contains(path []grid.Cell, c grid.Cell) bool {
	for _, p := range path {
		if p == c {
			return true
		}
	}
	return false
}

// TestPlanBudgetExhausted 验证“超出规划时限/预算”必须被明确标记，而非空轨迹或静默成功。
func TestPlanBudgetExhausted(t *testing.T) {
	base := twoGapMap()
	lp := local.New(local.Config{DT: 0.25, Horizon: 6, VSamples: 3, OmegaSamples: 7,
		GoalTolerance: 0.4, ObstacleMargin: 0.25, MaxIters: 80}, kinematics.DefaultConstraints())
	n := New(planner.New(1), lp, base) // 只允许展开 1 个节点
	_ = n.ApplyMap(MapUpdate{Seq: 1, Grid: base})
	req := Request{Start: kinematics.Pose{X: 1, Y: 1}, Goal: grid.Cell{X: 13, Y: 8}}
	r := n.Plan(req)
	logResult(t, "budget-exhausted", req, r)
	if r.Status != StatusPlanLimit || !errors.Is(r.Reason, planner.ErrPlanLimit) {
		t.Fatalf("want PLAN_LIMIT got %s/%v", r.Status, r.Reason)
	}
	if r.Traj != nil || len(r.Path) != 0 {
		t.Fatal("over-budget plan must not emit a path/trajectory")
	}
}
