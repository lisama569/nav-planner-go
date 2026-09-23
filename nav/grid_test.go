package nav

import "testing"

func TestGridCoordinateRoundTrip(t *testing.T) {
	m := NewGridMap(testConfig().Grid)
	cases := []struct {
		x, y float64
		c    Cell
	}{
		{0, 0, Cell{0, 0}},
		{0.5, 0.5, Cell{1, 1}},
		{1.0, 1.5, Cell{3, 2}},
	}
	for _, tc := range cases {
		got := m.WorldToCell(tc.x, tc.y)
		if got != tc.c {
			t.Fatalf("WorldToCell(%v,%v)=%s want %s", tc.x, tc.y, got, tc.c)
		}
		x, y := m.CellCenter(tc.c)
		if x != tc.x || y != tc.y {
			t.Fatalf("CellCenter(%s)=(%v,%v) want (%v,%v)", tc.c, x, y, tc.x, tc.y)
		}
	}
	// 越界栅格视为被占据。
	if !m.IsBlocked(Cell{-1, 0}) || !m.IsBlocked(Cell{99, 99}) {
		t.Fatal("out-of-bounds cells must count as blocked")
	}
}

func TestStampObstacleAndClone(t *testing.T) {
	cfg := testConfig()
	m := NewGridMap(cfg.Grid)
	// 中心 (1,1)、半宽 0.4 的方块 + 0 膨胀，应恰好覆盖中心栅格。
	if err := m.StampObstacle(Obstacle{ID: "o1", X: 1.0, Y: 1.0, HalfX: 0.4, HalfY: 0.4}); err != nil {
		t.Fatal(err)
	}
	if !m.IsBlocked(m.WorldToCell(1.0, 1.0)) {
		t.Fatal("center cell must be blocked after stamping")
	}
	if m.IsBlocked(m.WorldToCell(2.0, 2.0)) {
		t.Fatal("far cell must remain free")
	}
	clone := m.Clone()
	clone.SetBlocked(m.WorldToCell(2.0, 2.0), true)
	if m.IsBlocked(m.WorldToCell(2.0, 2.0)) {
		t.Fatal("mutating clone must not affect original map")
	}
	// 非法几何被分类。
	err := m.StampObstacle(Obstacle{ID: "bad", X: 1, Y: 1, HalfX: -1})
	if pe, ok := AsPlanningError(err); !ok || pe.Code != FailObstacleInvalid {
		t.Fatalf("want FailObstacleInvalid, got %v", err)
	}
}

func TestNewGridMapRejectsBadConfig(t *testing.T) {
	if NewGridMap(GridConfig{Rows: 0, Cols: 1, Resolution: 1}) != nil {
		t.Fatal("expected nil for zero rows")
	}
}
