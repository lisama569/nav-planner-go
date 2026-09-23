package nav

import "testing"

func buildTestTraj(t *testing.T, start Pose, goal Pose, m *GridMap, cfg Config) *Trajectory {
	t.Helper()
	path, err := NewGlobalPlanner(cfg).Search(m, start, goal, nil, nil)
	if err != nil {
		t.Fatal(err)
	}
	traj := NewTrajectoryBuilder(cfg).Build(start, goal, path, nil, nil)
	if !traj.Valid() {
		t.Fatalf("setup trajectory invalid: %s", traj.Reason)
	}
	return traj
}

func TestLocalAvoiderCleanAndConflict(t *testing.T) {
	cfg := testConfig()
	m := NewGridMap(cfg.Grid)
	start := Pose{X: 0.5, Y: 0.5}
	goal := Pose{X: 8.0, Y: 8.0}
	traj := buildTestTraj(t, start, goal, m, cfg)
	a := NewLocalAvoider(cfg)

	// 空世界：无冲突。
	v := a.Check(m, traj, start, nil)
	if v.Blocked {
		t.Fatalf("empty world must not block: %s", v.Reason)
	}
	t.Logf("clean: %s", v.Reason)

	// 在旧轨迹走廊上放障碍：必须 blocked 并给出可解释原因。
	m2 := m.Clone()
	obs := Obstacle{ID: "d", X: 1.5, Y: 1.5, HalfX: 0.24, HalfY: 0.24}
	if err := m2.StampObstacle(obs); err != nil {
		t.Fatal(err)
	}
	v = a.Check(m2, traj, start, map[string]Obstacle{"d": obs})
	if !v.Blocked {
		t.Fatalf("obstacle on corridor must block: %s", v.Reason)
	}
	if v.ConflictAt < 0 || v.ConflictArc < 0 || v.ConflictArc > cfg.LocalLookahead {
		t.Fatalf("bad conflict report: %+v", v)
	}
	t.Logf("conflict: %s", v.Reason)

	// 移除障碍（用无障碍世界）后必须恢复无冲突，结论可前后一致地翻转。
	if v2 := a.Check(m, traj, start, map[string]Obstacle{}); v2.Blocked {
		t.Fatalf("after removal corridor must be clean: %s", v2.Reason)
	}
}

func TestLocalAvoiderFarObstacleIgnored(t *testing.T) {
	cfg := testConfig()
	m := NewGridMap(cfg.Grid)
	start := Pose{X: 0.5, Y: 0.5}
	goal := Pose{X: 8.0, Y: 8.0}
	traj := buildTestTraj(t, start, goal, m, cfg)

	m2 := m.Clone()
	// 障碍在目标附近，远超 2m 前瞻窗；旧轨迹当前段不应判堵。
	obs := Obstacle{ID: "far", X: 7.5, Y: 7.5, HalfX: 0.24, HalfY: 0.24}
	if err := m2.StampObstacle(obs); err != nil {
		t.Fatal(err)
	}
	v := NewLocalAvoider(cfg).Check(m2, traj, start, map[string]Obstacle{"far": obs})
	if v.Blocked {
		t.Fatalf("obstacle outside lookahead must not block: %s", v.Reason)
	}
	t.Logf("far obstacle: %s", v.Reason)
}

func TestLocalAvoiderNilInputs(t *testing.T) {
	cfg := testConfig()
	a := NewLocalAvoider(cfg)
	if v := a.Check(nil, &Trajectory{Points: []TrajPoint{{}}}, Pose{}, nil); !v.Blocked {
		t.Fatal("nil map must block with explanation")
	}
	if v := a.Check(NewGridMap(cfg.Grid), nil, Pose{}, nil); !v.Blocked {
		t.Fatal("nil trajectory must block with explanation")
	}
}

// TestLocalAvoiderOnlyFutureCorridor 验证局部检查只评估当前位姿前方：
// 障碍落在机器人已经驶过的轨迹段上时不得触发重规划。
func TestLocalAvoiderOnlyFutureCorridor(t *testing.T) {
	cfg := testConfig()
	m := NewGridMap(cfg.Grid)
	start := Pose{X: 0.5, Y: 0.5}
	goal := Pose{X: 8.0, Y: 8.0}
	traj := buildTestTraj(t, start, goal, m, cfg)

	// 机器人已经沿走廊前进到 (2.5,2.5) 附近。
	robot := Pose{X: 2.5, Y: 2.5}

	// 身后障碍（起点附近，弧长早已驶过）：不得判堵。
	behind := m.Clone()
	obs := Obstacle{ID: "behind", X: 1.5, Y: 1.5, HalfX: 0.24, HalfY: 0.24}
	if err := behind.StampObstacle(obs); err != nil {
		t.Fatal(err)
	}
	if v := NewLocalAvoider(cfg).Check(behind, traj, robot, map[string]Obstacle{"behind": obs}); v.Blocked {
		t.Fatalf("obstacle behind robot must not trigger replan: %s", v.Reason)
	} else {
		t.Logf("behind obstacle ignored: %s", v.Reason)
	}

	// 前方障碍（机器人 2m 前瞻窗内）：必须判堵。
	ahead := m.Clone()
	obsA := Obstacle{ID: "ahead", X: 3.5, Y: 3.5, HalfX: 0.24, HalfY: 0.24}
	if err := ahead.StampObstacle(obsA); err != nil {
		t.Fatal(err)
	}
	if v := NewLocalAvoider(cfg).Check(ahead, traj, robot, map[string]Obstacle{"ahead": obsA}); !v.Blocked {
		t.Fatalf("obstacle ahead must trigger replan: %s", v.Reason)
	} else {
		t.Logf("ahead obstacle blocks: %s", v.Reason)
	}
}
