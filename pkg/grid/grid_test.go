package grid

import "testing"

func TestGridBasics(t *testing.T) {
	g := New(3, 2)
	if g == nil || g.Width() != 3 || g.Height() != 2 {
		t.Fatal("new grid")
	}
	c := Cell{X: 1, Y: 1}
	if !g.Free(c) {
		t.Fatal("new cell should be free")
	}
	g.Set(c, true)
	if !g.Occupied(c) {
		t.Fatal("set occupied failed")
	}
	// 越界与 nil 语义
	if g.Free(Cell{X: 9, Y: 9}) {
		t.Fatal("out-of-bounds must be treated occupied")
	}
	var nilGrid *Grid
	if nilGrid.InBounds(c) || nilGrid.Free(c) {
		t.Fatal("nil grid must be inBounds=false/free=false")
	}
	// 固定邻居顺序：上右下左
	got := g.Neighbors4(Cell{X: 1, Y: 0})
	want := []Cell{{X: 1, Y: 1}, {X: 2, Y: 0}, {X: 0, Y: 0}}
	if len(got) != len(want) {
		t.Fatalf("neighbors %v", got)
	}
	for i := range want {
		if got[i] != want[i] {
			t.Fatalf("neighbor order mismatch: %v want %v", got, want)
		}
	}
	t.Logf("[grid] 邻居顺序确定性=%v 越界/nil 安全=true", got)

	// 深拷贝隔离
	cp := g.Clone()
	cp.Set(c, false)
	if !g.Occupied(c) {
		t.Fatal("clone must not share backing storage")
	}
}

func TestNewRejectsBadSize(t *testing.T) {
	if New(0, 3) != nil || New(3, -1) != nil {
		t.Fatal("bad size must return nil")
	}
}
