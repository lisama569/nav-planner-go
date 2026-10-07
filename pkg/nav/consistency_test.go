package nav

import (
	"errors"
	"testing"

	"github.com/lisama569/nav-planner-go/pkg/grid"
	"github.com/lisama569/nav-planner-go/pkg/kinematics"
	"github.com/lisama569/nav-planner-go/pkg/local"
	"github.com/lisama569/nav-planner-go/pkg/planner"
)

// assertConsistent 校验两个入口对同一请求给出可互相解释的结论：
// 状态一致、路径一致，且成功类结论的轨迹都通过整轨迹障碍终检。
func assertConsistent(t *testing.T, name string, n *Navigator, req Request) (Result, Result) {
	t.Helper()
	rPlan := n.Plan(req)
	rStep := n.ReplanningStep(req)
	logResult(t, name+"/Plan", req, rPlan)
	logResult(t, name+"/ReplanningStep", req, rStep)
	if rPlan.Status != rStep.Status {
		t.Fatalf("[%s] 入口结论不一致: Plan=%s Step=%s", name, rPlan.Status, rStep.Status)
	}
	if !pathsEqual(rPlan.Path, rStep.Path) {
		t.Fatalf("[%s] 入口路径不一致:\n Plan=%v\n Step=%v", name, rPlan.Path, rStep.Path)
	}
	if errors.Is(rPlan.Reason, planner.ErrUnreachable) != errors.Is(rStep.Reason, planner.ErrUnreachable) {
		t.Fatalf("[%s] 不可达原因分类不一致: %v vs %v", name, rPlan.Reason, rStep.Reason)
	}
	for tag, r := range map[string]Result{"Plan": rPlan, "Step": rStep} {
		if r.Status == StatusPlanned || r.Status == StatusReplanned {
			g, obs := n.Snapshot()
			if r.Traj == nil {
				t.Fatalf("[%s/%s] 成功结论必须带可下发轨迹", name, tag)
			}
			if hit, col, id := n.lp.FirstCollision(r.Traj, g, obs); hit {
				t.Fatalf("[%s/%s] 下发轨迹终检碰撞: obstacle=%s %s", name, tag, id, col.Msg)
			}
		}
	}
	return rPlan, rStep
}

func pathsEqual(a, b []grid.Cell) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

// TestEntriesConsistentAcrossWorlds 覆盖：无障碍、动态障碍堵近路（绕行）、
// 两缺口全封（不可达）三种世界下，Plan 与 ReplanningStep 的结论与路线必须一致。
func TestEntriesConsistentAcrossWorlds(t *testing.T) {
	req := Request{Seq: 100, Tick: 1, Start: kinematics.Pose{X: 1, Y: 1}, Goal: grid.Cell{X: 13, Y: 8}}

	// 世界 1：仅静态地图，无动态障碍
	n1 := newTestNav(twoGapMap())
	_ = n1.ApplyMap(MapUpdate{Seq: 1, Grid: twoGapMap()})
	r1, _ := assertConsistent(t, "clean-world", n1, req)
	if r1.Status != StatusPlanned || !contains(r1.Path, grid.Cell{X: 7, Y: 4}) {
		t.Fatalf("clean world: want PLANNED via (7,4), got %s %v", r1.Status, r1.Path)
	}

	// 世界 2：障碍堵住近缺口 => 两入口都必须绕行 (7,0)，绝不能给出穿过障碍的路线
	n2 := newTestNav(twoGapMap())
	_ = n2.ApplyMap(MapUpdate{Seq: 1, Grid: twoGapMap()})
	if err := n2.ApplyObstacle(ObstacleUpdate{Seq: 2, Obstacle: local.Obstacle{
		ID: "m", X: 7, Y: 4, Radius: 0.7, Seq: 2}}); err != nil {
		t.Fatal(err)
	}
	r2, _ := assertConsistent(t, "moving-obstacle", n2, req)
	if r2.Status != StatusReplanned {
		t.Fatalf("moving obstacle: want REPLANNED got %s (%v)", r2.Status, r2.Reason)
	}
	if contains(r2.Path, grid.Cell{X: 7, Y: 4}) || !contains(r2.Path, grid.Cell{X: 7, Y: 0}) {
		t.Fatalf("moving obstacle: both entries must detour via (7,0): %v", r2.Path)
	}

	// 世界 3：两个缺口都封死 => 两入口都 UNREACHABLE
	n3 := newTestNav(twoGapMap())
	_ = n3.ApplyMap(MapUpdate{Seq: 1, Grid: twoGapMap()})
	_ = n3.ApplyObstacle(ObstacleUpdate{Seq: 2, Obstacle: local.Obstacle{ID: "m", X: 7, Y: 4, Radius: 0.7, Seq: 2}})
	_ = n3.ApplyObstacle(ObstacleUpdate{Seq: 3, Obstacle: local.Obstacle{ID: "s", X: 7, Y: 0, Radius: 0.7, Seq: 3}})
	r3, _ := assertConsistent(t, "both-gaps-sealed", n3, req)
	if r3.Status != StatusUnreachable || !errors.Is(r3.Reason, planner.ErrUnreachable) {
		t.Fatalf("sealed world: want UNREACHABLE/ErrUnreachable got %s/%v", r3.Status, r3.Reason)
	}
	if r3.Traj != nil || len(r3.Path) != 0 {
		t.Fatal("unreachable must not emit path/trajectory")
	}
}

// TestEntriesConsistentRejections 验证非法输入在两个入口下都被一致拒绝。
func TestEntriesConsistentRejections(t *testing.T) {
	n := newTestNav(twoGapMap())
	_ = n.ApplyMap(MapUpdate{Seq: 1, Grid: twoGapMap()})
	start := kinematics.Pose{X: 1, Y: 1}
	for _, c := range []struct {
		name string
		req  Request
		want error
	}{
		{"goal-oob", Request{Start: start, Goal: grid.Cell{X: 99, Y: 99}}, planner.ErrGoalOutOfBounds},
		{"goal-occupied", Request{Start: start, Goal: grid.Cell{X: 7, Y: 1}}, planner.ErrGoalOccupied},
	} {
		rp, _ := assertConsistent(t, c.name, n, c.req)
		if !errors.Is(rp.Reason, c.want) {
			t.Fatalf("%s: want %v got %v", c.name, c.want, rp.Reason)
		}
	}
}
