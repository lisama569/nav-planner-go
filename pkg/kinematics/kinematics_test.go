package kinematics

import (
	"testing"

	"github.com/lisama569/nav-planner-go/pkg/grid"
)

func samplePath() []grid.Cell {
	return []grid.Cell{{X: 1, Y: 1}, {X: 2, Y: 1}, {X: 2, Y: 2}, {X: 3, Y: 2}}
}

func TestPathToTrajectoryValid(t *testing.T) {
	c := DefaultConstraints()
	start := Pose{X: 1, Y: 1, Theta: 0}
	traj, err := PathToTrajectory(samplePath(), start, c, 0.25)
	if err != nil {
		t.Fatalf("build: %v", err)
	}
	vs := Validate(traj, start, c)
	t.Logf("[traj-valid] 段数=%d 时长=%.3f 末端=%+v 违规数=%d",
		len(traj.Points), traj.Duration(), traj.End(), len(vs))
	for _, v := range vs {
		t.Logf("  违规: %v", v)
	}
	if len(vs) != 0 {
		t.Fatalf("constructed trajectory must satisfy constraints: %v", vs)
	}
	end := traj.End()
	if abs(end.X-3) > 1e-7 || abs(end.Y-2) > 1e-7 {
		t.Fatalf("end pose off: %+v", end)
	}
}

func TestPathToTrajectoryErrors(t *testing.T) {
	c := DefaultConstraints()
	if _, err := PathToTrajectory([]grid.Cell{{X: 1, Y: 1}}, Pose{}, c, 0.25); err != ErrShortPath {
		t.Fatalf("want ErrShortPath got %v", err)
	}
	if _, err := PathToTrajectory(samplePath(), Pose{}, c, 0); err != ErrBadTimestep {
		t.Fatalf("want ErrBadTimestep got %v", err)
	}
	if _, err := PathToTrajectory(samplePath(), Pose{}, Constraints{}, 0.25); err != ErrBadConstraints {
		t.Fatalf("want ErrBadConstraints got %v", err)
	}
	t.Log("[traj-errors] 短路径/非正步长/非法约束三类错误均被正确拒绝")
}

func TestValidateViolationClasses(t *testing.T) {
	c := DefaultConstraints()
	start := Pose{}
	cases := map[string]*Trajectory{
		"velocity":      {Points: []Point{{Pose: Pose{X: 5, Y: 0}, V: 9, Omega: 0, DT: 1}}},
		"omega":         {Points: []Point{{Pose: Pose{X: 0, Y: 0, Theta: 3}, V: 0, Omega: 3, DT: 1}}},
		"timing":        {Points: []Point{{Pose: Pose{X: 1, Y: 0}, V: 1, DT: 0}}},
		"curvature":     {Points: []Point{{Pose: Pose{X: 0.001, Y: 0}, V: 0.001, Omega: 1.57, DT: 0.01}}},
		"discontinuity": {Points: []Point{{Pose: Pose{X: 2, Y: 2}, V: 1, DT: 1}}},
	}
	for name, tr := range cases {
		vs := Validate(tr, start, c)
		t.Logf("[violation:%s] 输入1段 违规数=%d 首类=%s", name, len(vs), vs[0].Code)
		if len(vs) == 0 {
			t.Fatalf("%s: expected violation", name)
		}
	}
	// turn-angle: 单段转角超过 90° 且积分位姿一致时只报转角
	turn := &Trajectory{Points: []Point{{Pose: Pose{X: 0, Y: 0, Theta: 2.5}, V: 0, Omega: 2.5, DT: 1}}}
	vs := Validate(turn, start, c)
	t.Logf("[violation:turn] 违规=%v", vs)
	found := false
	for _, v := range vs {
		if v.Code == ViolationTurnAngle {
			found = true
		}
	}
	if !found {
		t.Fatal("expected turn-angle violation")
	}
}

func TestEmptyTrajectoryRejected(t *testing.T) {
	vs := Validate(nil, Pose{}, DefaultConstraints())
	t.Logf("[empty-traj] 违规=%v", vs)
	if len(vs) == 0 || vs[0].Code != ViolationTiming {
		t.Fatalf("empty trajectory must be flagged, got %v", vs)
	}
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}
