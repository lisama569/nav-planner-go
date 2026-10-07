package nav

import (
	"errors"
	"testing"

	"github.com/lisama569/nav-planner-go/pkg/grid"
	"github.com/lisama569/nav-planner-go/pkg/kinematics"
	"github.com/lisama569/nav-planner-go/pkg/local"
	"github.com/lisama569/nav-planner-go/pkg/planner"
)

// timeoutNav 构造一个墙上时钟时限极短（虚拟时钟驱动）的编排器，
// 展开预算放得很宽，保证先触发 TIMEOUT 而非 PLAN_LIMIT。
func timeoutNav(t *testing.T, g *grid.Grid) *Navigator {
	t.Helper()
	lp := local.New(local.Config{DT: 0.25, Horizon: 6, VSamples: 3, OmegaSamples: 7,
		GoalTolerance: 0.4, ObstacleMargin: 0.25, MaxIters: 80}, kinematics.DefaultConstraints())
	gp := planner.NewWithLimit(planner.Limit{
		MaxExpansions: 1_000_000, DeadlineMicros: 2000,
		Clock: planner.NewManualClock(1000), // 每 256 次展开读一次 => 512 次展开超时
	})
	n := New(gp, lp, g)
	if err := n.ApplyMap(MapUpdate{Seq: 1, Grid: g}); err != nil {
		t.Fatal(err)
	}
	return n
}

// TestNavTimeoutClassified 验证超大地图下墙上时钟超时得到独立可区分的 TIMEOUT，
// 且两个入口结论一致、绝不下发半成品路径/轨迹。
func TestNavTimeoutClassified(t *testing.T) {
	g := grid.New(200, 200)
	n := timeoutNav(t, g)
	req := Request{Seq: 1, Tick: 1, Start: kinematics.Pose{X: 1, Y: 1}, Goal: grid.Cell{X: 198, Y: 198}}

	rp := n.Plan(req)
	rs := n.ReplanningStep(req)
	logResult(t, "timeout/Plan", req, rp)
	logResult(t, "timeout/ReplanningStep", req, rs)
	for tag, r := range map[string]Result{"Plan": rp, "Step": rs} {
		if r.Status != StatusTimeout {
			t.Fatalf("%s: want TIMEOUT got %s (%v)", tag, r.Status, r.Reason)
		}
		if !errors.Is(r.Reason, planner.ErrPlanTimeout) {
			t.Fatalf("%s: reason must be ErrPlanTimeout got %v", tag, r.Reason)
		}
		if errors.Is(r.Reason, planner.ErrPlanLimit) {
			t.Fatalf("%s: TIMEOUT 不得与 PLAN_LIMIT 混淆", tag)
		}
		if r.Traj != nil || len(r.Path) != 0 {
			t.Fatalf("%s: 超时不得下发半成品", tag)
		}
		t.Logf("[timeout/%s] 判定依据: 虚拟时钟在 expansions=%d 处越过 2000us 时限 => TIMEOUT（独立于 PLAN_LIMIT）",
			tag, r.Expansions)
	}
}

// TestNavTimeoutDeterministic 验证带虚拟时钟时限的同一份输入，
// 在全新编排器上多次运行结论与触发展开数完全一致（确定性重放边界）。
func TestNavTimeoutDeterministic(t *testing.T) {
	req := Request{Seq: 1, Tick: 1, Start: kinematics.Pose{X: 1, Y: 1}, Goal: grid.Cell{X: 198, Y: 198}}
	run := func() Result {
		n := timeoutNav(t, grid.New(200, 200))
		return n.Plan(req)
	}
	a, b := run(), run()
	t.Logf("[timeout-determinism] expansions=%d,%d status=%s,%s",
		a.Expansions, b.Expansions, a.Status, b.Status)
	if a.Status != StatusTimeout || b.Status != StatusTimeout {
		t.Fatalf("both must time out: %s %s", a.Status, b.Status)
	}
	if a.Expansions != b.Expansions {
		t.Fatalf("timeout trigger must be stable across runs: %d vs %d", a.Expansions, b.Expansions)
	}
}

// TestPlanLimitStillClassified 回归：展开预算耗尽仍然是 PLAN_LIMIT，
// 新增 TIMEOUT 分类不改变既有预算语义。
func TestPlanLimitStillClassified(t *testing.T) {
	base := twoGapMap()
	lp := local.New(local.Config{DT: 0.25, Horizon: 6, VSamples: 3, OmegaSamples: 7,
		GoalTolerance: 0.4, ObstacleMargin: 0.25, MaxIters: 80}, kinematics.DefaultConstraints())
	n := New(planner.New(1), lp, base) // 只允许展开 1 个节点，且未配置墙上时钟
	_ = n.ApplyMap(MapUpdate{Seq: 1, Grid: base})
	req := Request{Start: kinematics.Pose{X: 1, Y: 1}, Goal: grid.Cell{X: 13, Y: 8}}
	r := n.Plan(req)
	logResult(t, "plan-limit-regression", req, r)
	if r.Status != StatusPlanLimit || !errors.Is(r.Reason, planner.ErrPlanLimit) {
		t.Fatalf("want PLAN_LIMIT/ErrPlanLimit got %s/%v", r.Status, r.Reason)
	}
	if errors.Is(r.Reason, planner.ErrPlanTimeout) {
		t.Fatal("budget limit must not report timeout")
	}
}
