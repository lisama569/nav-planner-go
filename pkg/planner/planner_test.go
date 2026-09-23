package planner

import (
	"errors"
	"testing"

	"github.com/lisama569/nav-planner-go/pkg/grid"
)

// logCase 统一打印输入、结果与判定依据，满足可解释性要求。
func logCase(t *testing.T, name string, start, goal grid.Cell, path []grid.Cell, err error, want error) {
	t.Helper()
	t.Logf("[%s] 输入 start=%v goal=%v", name, start, goal)
	if err != nil {
		t.Logf("[%s] 结果 error=%v | 判定依据 期望=%v 匹配=%v", name, err, want, errors.Is(err, want))
	} else {
		t.Logf("[%s] 结果 path(%d)=%v expansions=%d | 判定依据 期望=成功 匹配=%v",
			name, len(path), path, 0, want == nil)
	}
}

func wallMap() *grid.Grid {
	g := grid.New(8, 6)
	for y := 0; y < 6; y++ {
		if y == 2 {
			continue // 唯一缺口 (4,2)
		}
		g.Set(grid.Cell{X: 4, Y: y}, true)
	}
	return g
}

func TestPlanSuccessPath(t *testing.T) {
	g := wallMap()
	p := New(0)
	start, goal := grid.Cell{X: 1, Y: 1}, grid.Cell{X: 6, Y: 4}
	path, err := p.Plan(g, start, goal)
	logCase(t, "success", start, goal, path, err, nil)
	if err != nil {
		t.Fatalf("expected success, got %v", err)
	}
	if path[0] != start || path[len(path)-1] != goal {
		t.Fatalf("path endpoints wrong: %v..%v", path[0], path[len(path)-1])
	}
	// 路径必须经过唯一缺口
	found := false
	for _, c := range path {
		if c == (grid.Cell{X: 4, Y: 2}) {
			found = true
		}
		if g.Occupied(c) {
			t.Fatalf("path steps on occupied cell %v", c)
		}
	}
	if !found {
		t.Fatalf("path must pass through gap (4,2): %v", path)
	}
	// 路径步长必须为单位移动（四邻域）
	for i := 0; i+1 < len(path); i++ {
		dx, dy := path[i+1].X-path[i].X, path[i+1].Y-path[i].Y
		if (dx == 1 || dx == -1) != (dy == 0) || (dy == 1 || dy == -1) != (dx == 0) {
			t.Fatalf("non-grid-adjacent step %v->%v", path[i], path[i+1])
		}
	}
}

func TestPlanDeterminism(t *testing.T) {
	g := wallMap()
	start, goal := grid.Cell{X: 1, Y: 1}, grid.Cell{X: 6, Y: 4}
	p1, _ := New(0).Plan(g, start, goal)
	p2, _ := New(0).Plan(g.Clone(), start, goal)
	t.Logf("[determinism] 两次规划路径相等: %v", cellsEqual(p1, p2))
	if !cellsEqual(p1, p2) {
		t.Fatalf("A* not deterministic:\n%v\n%v", p1, p2)
	}
}

func TestPlanRejectionClasses(t *testing.T) {
	g := wallMap()
	cases := []struct {
		name  string
		gg    *grid.Grid
		start grid.Cell
		goal  grid.Cell
		want  error
	}{
		{"map missing", nil, grid.Cell{X: 1, Y: 1}, grid.Cell{X: 6, Y: 4}, ErrMapMissing},
		{"start out of bounds", g, grid.Cell{X: -1, Y: 1}, grid.Cell{X: 6, Y: 4}, ErrStartOutOfBounds},
		{"goal out of bounds", g, grid.Cell{X: 1, Y: 1}, grid.Cell{X: 66, Y: 4}, ErrGoalOutOfBounds},
		{"start occupied", g, grid.Cell{X: 4, Y: 0}, grid.Cell{X: 6, Y: 4}, ErrStartOccupied},
		{"goal occupied", g, grid.Cell{X: 1, Y: 1}, grid.Cell{X: 4, Y: 0}, ErrGoalOccupied},
		{"unreachable", sealedMap(), grid.Cell{X: 1, Y: 1}, grid.Cell{X: 6, Y: 4}, ErrUnreachable},
		{"budget exhausted", g, grid.Cell{X: 1, Y: 1}, grid.Cell{X: 6, Y: 4}, ErrPlanLimit},
	}
	for _, c := range cases {
		pp := New(0)
		if c.name == "budget exhausted" {
			pp = New(1) // 展开 1 个节点即超预算
		}
		path, err := pp.Plan(c.gg, c.start, c.goal)
		logCase(t, c.name, c.start, c.goal, path, err, c.want)
		if !errors.Is(err, c.want) {
			t.Fatalf("%s: want %v got %v", c.name, c.want, err)
		}
	}
}

func TestStartEqualsGoal(t *testing.T) {
	g := wallMap()
	c := grid.Cell{X: 1, Y: 1}
	path, err := New(0).Plan(g, c, c)
	logCase(t, "start==goal", c, c, path, err, nil)
	if err != nil || len(path) != 1 || path[0] != c {
		t.Fatalf("expected single-cell path, got %v %v", path, err)
	}
}

func sealedMap() *grid.Grid {
	g := grid.New(8, 6)
	for y := 0; y < 6; y++ {
		g.Set(grid.Cell{X: 4, Y: y}, true) // 无缺口的墙
	}
	return g
}

func cellsEqual(a, b []grid.Cell) bool {
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
