// Package grid 提供确定性的二维占据栅格地图模型。
//
// 坐标约定：整数栅格坐标 (X,Y)，原点在左下角，X 向右、Y 向上。
// 地图按行优先（Y 外层、X 内层）线性存储，Clone 为深拷贝，
// 保证并发更新与快照读取不会共享底层数组。
package grid

import (
	"errors"
	"fmt"
)

// 地图相关的可区分错误。
var (
	// ErrMapMissing 表示尚未装载任何地图。
	ErrMapMissing = errors.New("grid: map is missing")

	// ErrInvalidSize 表示请求构造尺寸非法的地图。
	ErrInvalidSize = errors.New("grid: width and height must be positive")
)

// Cell 是栅格坐标（整数）。
type Cell struct {
	X, Y int
}

// String 返回可读坐标，用于日志与判定依据打印。
func (c Cell) String() string { return fmt.Sprintf("(%d,%d)", c.X, c.Y) }

// Grid 是行优先存储的占据栅格，true 表示被占据（障碍）。
type Grid struct {
	width    int
	height   int
	occupied []bool
}

// New 创建 width*height 的空地图（全部自由）。
func New(width, height int) *Grid {
	if width <= 0 || height <= 0 {
		return nil
	}
	return &Grid{
		width:    width,
		height:   height,
		occupied: make([]bool, width*height),
	}
}

// Width 返回地图宽度。
func (g *Grid) Width() int { return g.width }

// Height 返回地图高度。
func (g *Grid) Height() int { return g.height }

func (g *Grid) index(c Cell) int { return c.Y*g.width + c.X }

// InBounds 判断单元格是否在地图范围内；nil 地图返回 false。
func (g *Grid) InBounds(c Cell) bool {
	if g == nil {
		return false
	}
	return c.X >= 0 && c.X < g.width && c.Y >= 0 && c.Y < g.height
}

// Occupied 判断单元格是否被占据，nil 地图或越界均视为被占据，
// 这样规划器在地图缺失时不可能“静默穿过”不存在的区域。
func (g *Grid) Occupied(c Cell) bool {
	if !g.InBounds(c) {
		return true
	}
	return g.occupied[g.index(c)]
}

// Free 是 Occupied 的反向语义。
func (g *Grid) Free(c Cell) bool {
	return !g.Occupied(c)
}

// Set 设置单元格的占据状态；越界写入被忽略（由调用方先校验）。
func (g *Grid) Set(c Cell, occupied bool) {
	if !g.InBounds(c) {
		return
	}
	g.occupied[g.index(c)] = occupied
}

// Neighbors4 返回四邻域中位于地图内的单元格，顺序固定（上右下左），
// 固定顺序是 A* 平局结果可复现的前提之一。
func (g *Grid) Neighbors4(c Cell) []Cell {
	if g == nil {
		return nil
	}
	order := [4]Cell{{c.X, c.Y + 1}, {c.X + 1, c.Y}, {c.X, c.Y - 1}, {c.X - 1, c.Y}}
	out := make([]Cell, 0, 4)
	for _, n := range order {
		if g.InBounds(n) {
			out = append(out, n)
		}
	}
	return out
}

// Clone 返回地图的深拷贝；nil 返回 nil。
func (g *Grid) Clone() *Grid {
	if g == nil {
		return nil
	}
	cp := &Grid{width: g.width, height: g.height, occupied: make([]bool, len(g.occupied))}
	copy(cp.occupied, g.occupied)
	return cp
}
