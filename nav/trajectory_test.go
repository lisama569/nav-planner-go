package nav

import (
	"math"
	"testing"
)

func TestBuildTrajectorySatisfiesConstraints(t *testing.T) {
	cfg := testConfig()
	m := NewGridMap(cfg.Grid)
	gp := NewGlobalPlanner(cfg)
	start := Pose{X: 1.0, Y: 1.0, Theta: 0.3}
	goal := Pose{X: 4.0, Y: 3.0, Theta: -1.0}
	path, err := gp.Search(m, start, goal, nil, nil)
	if err != nil {
		t.Fatal(err)
	}
	tb := NewTrajectoryBuilder(cfg)
	traj := tb.Build(start, goal, path, nil, nil)
	if !traj.Valid() {
		t.Fatalf("trajectory not valid: status=%d reason=%s", traj.Status, traj.Reason)
	}
	// Validate 再跑一次（独立入口，必须通过）。
	if reason := tb.Validate(traj); reason != "" {
		t.Fatalf("validate failed: %s", reason)
	}
	// 起止位姿与到达精度。
	first, last := traj.Points[0], traj.Points[len(traj.Points)-1]
	if first.Pose != start {
		t.Fatalf("first point %+v != start %+v", first.Pose, start)
	}
	if d := math.Hypot(last.Pose.X-goal.X, last.Pose.Y-goal.Y); d > 1e-6 {
		t.Fatalf("end position %+v not at goal, dist=%v", last.Pose, d)
	}
	if d := math.Abs(wrapAngle(last.Pose.Theta - goal.Theta)); d > 1e-6 {
		t.Fatalf("end heading %v != goal %v, diff=%v", last.Pose.Theta, goal.Theta, d)
	}
	// 末端必须真正停下。
	if math.Abs(last.LinearVel) > 1e-9 || math.Abs(last.AngularVel) > 1e-9 {
		t.Fatalf("robot must stop at goal: v=%v w=%v", last.LinearVel, last.AngularVel)
	}
	t.Logf("built valid trajectory: %d points, duration=%v", len(traj.Points), last.Time)
}

func TestBuildTrajectoryTimeout(t *testing.T) {
	cfg := testConfig()
	m := NewGridMap(cfg.Grid)
	gp := NewGlobalPlanner(cfg)
	path, err := gp.Search(m, Pose{X: 0, Y: 0}, Pose{X: 5, Y: 5}, nil, nil)
	if err != nil {
		t.Fatal(err)
	}
	clk := NewFakeClock(mustTime("2026-09-23T00:00:00Z"))
	deadline := clk.Now() // 一开始就过期
	traj := NewTrajectoryBuilder(cfg).Build(Pose{X: 0, Y: 0}, Pose{X: 5, Y: 5}, path, &deadline, clk)
	if traj.Status != TrajTimeout {
		t.Fatalf("want TrajTimeout, got status=%d reason=%s", traj.Status, traj.Reason)
	}
	if traj.Valid() {
		t.Fatal("timed-out partial trajectory must not be valid/dispatchable")
	}
	t.Logf("timeout trajectory correctly marked: %s", traj.Reason)
}

func TestValidateDetectsConstraintViolation(t *testing.T) {
	cfg := testConfig()
	tb := NewTrajectoryBuilder(cfg)
	dt := cfg.DT.Seconds()
	// 手工构造一个超速轨迹。
	bad := &Trajectory{Points: []TrajPoint{
		{Pose: Pose{}, Time: 0, LinearVel: 0},
		{Pose: Pose{X: cfg.Limits.MaxLinearVel * dt * 10}, Time: cfg.DT,
			LinearVel: cfg.Limits.MaxLinearVel * 10},
	}}
	reason := tb.Validate(bad)
	if reason == "" {
		t.Fatal("overspeed trajectory must fail validation")
	}
	t.Logf("violation classified: %s", reason)

	// 位姿不连续的轨迹也必须被识别。
	discontinuous := &Trajectory{Points: []TrajPoint{
		{Pose: Pose{}, Time: 0, LinearVel: 0},
		{Pose: Pose{X: 9, Y: 9}, Time: cfg.DT, LinearVel: 0},
	}}
	if r := tb.Validate(discontinuous); r == "" {
		t.Fatal("pose-discontinuous trajectory must fail validation")
	} else {
		t.Logf("discontinuity classified: %s", r)
	}
}
