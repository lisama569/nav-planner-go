package nav

import "testing"

func newTestNavigator(t *testing.T) (*Navigator, *FakeClock) {
	t.Helper()
	clk := NewFakeClock(mustTime("2026-09-23T00:00:00Z"))
	n, err := NewNavigator(testConfig(), clk)
	if err != nil {
		t.Fatal(err)
	}
	m := NewGridMap(testConfig().Grid)
	if err := n.SetMap(m, nil); err != nil {
		t.Fatal(err)
	}
	return n, clk
}

func TestNavigatorHappyPath(t *testing.T) {
	n, _ := newTestNavigator(t)
	start := Pose{X: 1.0, Y: 1.0}
	goal := Pose{X: 4.0, Y: 4.0, Theta: 0.5}
	r := n.Plan(start, goal)
	logResult(t, "happy", start, goal, r)
	assertCode(t, r, FailNone)
	if !r.Succeeded() {
		t.Fatal("first plan must succeed with dispatchable trajectory")
	}
	snap := n.State()
	if !snap.HasMap || snap.Seq != 0 || snap.LastResult == nil {
		t.Fatalf("unexpected snapshot: %+v", snap)
	}
}

func TestNavigatorMapMissingAndBadPoses(t *testing.T) {
	clk := NewFakeClock(mustTime("2026-09-23T00:00:00Z"))
	n, err := NewNavigator(testConfig(), clk)
	if err != nil {
		t.Fatal(err)
	}
	r := n.Plan(Pose{X: 1, Y: 1}, Pose{X: 2, Y: 2})
	logResult(t, "no-map", Pose{X: 1, Y: 1}, Pose{X: 2, Y: 2}, r)
	assertCode(t, r, FailNotInitialized)

	n2, _ := newTestNavigator(t)
	r = n2.Plan(Pose{X: 1, Y: 1}, Pose{X: 99, Y: 99})
	logResult(t, "bad-goal", Pose{X: 1, Y: 1}, Pose{X: 99, Y: 99}, r)
	assertCode(t, r, FailGoalInvalid)

	// 起点落在静态障碍上。
	n3, _ := newTestNavigator(t)
	if ok, _ := n3.ApplyObstacle(Obstacle{ID: "s1", X: 1.0, Y: 1.0,
		HalfX: 0.25, HalfY: 0.25, Seq: 1}); !ok {
		t.Fatal("obstacle update should be accepted")
	}
	r = n3.Plan(Pose{X: 1.0, Y: 1.0}, Pose{X: 4, Y: 4})
	logResult(t, "blocked-start", Pose{X: 1.0, Y: 1.0}, Pose{X: 4, Y: 4}, r)
	assertCode(t, r, FailStartInvalid)
}

func TestNavigatorDynamicObstacleTriggersReplan(t *testing.T) {
	n, clk := newTestNavigator(t)
	// 目标选在远处，使第一次规划产生较长轨迹。
	start := Pose{X: 0.5, Y: 0.5}
	goal := Pose{X: 8.0, Y: 8.0}
	r1 := n.Plan(start, goal)
	logResult(t, "before-obstacle", start, goal, r1)
	assertCode(t, r1, FailNone)

	// 新增动态障碍，恰好落在起点前方（旧轨迹的早期走廊上）。
	obs := Obstacle{
		ID: "door", X: 1.5, Y: 1.5, HalfX: 0.24, HalfY: 0.24, Seq: 1,
	}
	accepted, why := n.ApplyObstacle(obs)
	if !accepted {
		t.Fatalf("obstacle should be accepted: %s", why)
	}
	clk.Advance(0)
	// 从同一当前位姿、同一目标再次规划 => 必须是重规划。
	r2 := n.Plan(start, goal)
	logResult(t, "after-obstacle", start, goal, r2)
	assertCode(t, r2, FailNone)
	if !r2.Replanned {
		t.Fatalf("expected Replanned=true, reason=%q", r2.ReplanReason)
	}
	if r2.Seq != 1 {
		t.Fatalf("expected world seq 1, got %d", r2.Seq)
	}
	// 新路径不得经过障碍中心栅格 (r=3,c=3)（世界 (1.5,1.5)）。
	for _, node := range r2.Path.Nodes {
		if node.C == (Cell{Row: 3, Col: 3}) {
			t.Fatalf("replanned path must avoid obstacle cell %s", node.C)
		}
	}
	t.Logf("replanned path avoids obstacle, %d nodes", len(r2.Path.Nodes))

	// 移除障碍物（更大序号），世界版本推进，仍应规划成功。
	accepted, _ = n.ApplyObstacle(Obstacle{ID: "door", Seq: 2, Removed: true})
	if !accepted {
		t.Fatal("removal update should be accepted")
	}
	r3 := n.Plan(start, goal)
	logResult(t, "after-removal", start, goal, r3)
	assertCode(t, r3, FailNone)
	if r3.Seq != 2 {
		t.Fatalf("expected world seq 2, got %d", r3.Seq)
	}
}

func TestNavigatorStaleAndOutOfOrderUpdates(t *testing.T) {
	n, _ := newTestNavigator(t)
	if ok, _ := n.ApplyObstacle(Obstacle{ID: "a", Seq: 5, X: 1, Y: 1, HalfX: 0.2, HalfY: 0.2}); !ok {
		t.Fatal("seq=5 should be accepted")
	}
	// 乱序但更新：seq=10 先到。
	if ok, _ := n.ApplyObstacle(Obstacle{ID: "a", Seq: 10, X: 2, Y: 2, HalfX: 0.2, HalfY: 0.2}); !ok {
		t.Fatal("seq=10 should be accepted")
	}
	// 迟到的 seq=7 必须被丢弃。
	if ok, why := n.ApplyObstacle(Obstacle{ID: "a", Seq: 7, X: 3, Y: 3, HalfX: 0.2, HalfY: 0.2}); ok {
		t.Fatalf("stale seq=7 must be dropped, got: %s", why)
	} else {
		t.Logf("stale update correctly rejected: %s", why)
	}
	// 重复 seq=10 幂等丢弃。
	if ok, _ := n.ApplyObstacle(Obstacle{ID: "a", Seq: 10, X: 2, Y: 2, HalfX: 0.2, HalfY: 0.2}); ok {
		t.Fatal("duplicate seq=10 must be ignored")
	}
	// 世界版本只应被接受的更新推进两次。
	if n.State().Seq != 2 {
		t.Fatalf("expected seq 2, got %d", n.State().Seq)
	}
	// 障碍位置应停留在 seq=10 的 (2,2)。
	r := n.Plan(Pose{X: 0.5, Y: 0.5}, Pose{X: 8, Y: 8})
	logResult(t, "ordering", Pose{X: 0.5, Y: 0.5}, Pose{X: 8, Y: 8}, r)
	assertCode(t, r, FailNone)
}

func TestNavigatorGoalBecomesUnreachable(t *testing.T) {
	n, _ := newTestNavigator(t)
	start := Pose{X: 4.5, Y: 0.5}
	goal := Pose{X: 4.5, Y: 8.5}
	r1 := n.Plan(start, goal)
	logResult(t, "reachable", start, goal, r1)
	assertCode(t, r1, FailNone)

	// 横跨整行的墙把地图一分为二（留 0 缺口）。
	for col := 0; col < 20; col++ {
		ok, _ := n.ApplyObstacle(Obstacle{
			ID:    "wall-" + itoaInt(col),
			X:     0.25 + 0.5*float64(col),
			Y:     4.0,
			HalfX: 0.25,
			HalfY: 0.25,
			Seq:   int64(col + 1),
		})
		if !ok {
			t.Fatalf("wall segment %d rejected", col)
		}
	}
	// 从墙南侧的当前位姿再次规划到北侧目标 => 必须明确不可达。
	cur := Pose{X: 4.5, Y: 3.5}
	r2 := n.Plan(cur, goal)
	logResult(t, "unreachable", cur, goal, r2)
	assertCode(t, r2, FailUnreachable)
	if r2.Succeeded() || r2.Traj != nil && r2.Traj.Valid() {
		t.Fatal("unreachable goal must not yield a valid trajectory")
	}
}

func TestNavigatorDeterministicReplay(t *testing.T) {
	mk := func() (*Navigator, PlanResult) {
		n, _ := newTestNavigator(&testing.T{})
		n.ApplyObstacle(Obstacle{ID: "d1", Seq: 1, X: 2.0, Y: 2.0, HalfX: 0.24, HalfY: 0.24})
		n.ApplyObstacle(Obstacle{ID: "d2", Seq: 1, X: 5.0, Y: 6.0, HalfX: 0.24, HalfY: 0.24})
		return n, *n.Plan(Pose{X: 0.5, Y: 0.5}, Pose{X: 8.5, Y: 8.5})
	}
	n1, a := mk()
	n2, b := mk()
	if !equalResults(a, b) {
		t.Fatal("replay produced different results")
	}
	if n1.State().Seq != n2.State().Seq {
		t.Fatal("seq differs across replays")
	}
	t.Logf("deterministic replay: %d traj points, seq=%d", len(a.Traj.Points), a.Seq)
}

func equalResults(a, b PlanResult) bool {
	if a.Seq != b.Seq || len(a.Traj.Points) != len(b.Traj.Points) || len(a.Path.Nodes) != len(b.Path.Nodes) {
		return false
	}
	for i := range a.Traj.Points {
		if a.Traj.Points[i] != b.Traj.Points[i] {
			return false
		}
	}
	for i := range a.Path.Nodes {
		if a.Path.Nodes[i] != b.Path.Nodes[i] {
			return false
		}
	}
	return true
}

func itoaInt(v int) string { return itoa64(int64(v)) }
