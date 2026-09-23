package nav

import "math"

// GridMap 是带障碍标记的二维栅格地图。
//
// 并发约定：地图本身是“安装后不可变”的对象；Navigator 在每次规划时
// 克隆基础地图并叠加最新动态障碍，因此搜索过程无需持锁，也不会读到
// 被并发改写的中间状态。
type GridMap struct {
	cfg     GridConfig
	blocked [][]bool
}

// NewGridMap 根据配置创建空（全自由）栅格地图。
// 非法配置返回 nil（应由 Config.Validate 在更上层拦截）。
func NewGridMap(cfg GridConfig) *GridMap {
	if cfg.Rows <= 0 || cfg.Cols <= 0 || cfg.Resolution <= 0 {
		return nil
	}
	m := &GridMap{cfg: cfg}
	m.blocked = make([][]bool, cfg.Rows)
	for r := range m.blocked {
		m.blocked[r] = make([]bool, cfg.Cols)
	}
	return m
}

// Config 返回地图的几何配置。
func (m *GridMap) Config() GridConfig { return m.cfg }

// InBounds 判断栅格是否在地图范围内。
func (m *GridMap) InBounds(c Cell) bool {
	return m != nil && c.Row >= 0 && c.Row < m.cfg.Rows && c.Col >= 0 && c.Col < m.cfg.Cols
}

// IsBlocked 判断栅格是否被占据。越界栅格一律视为被占据，
// 这样搜索算法不需要单独处理越界邻居。
func (m *GridMap) IsBlocked(c Cell) bool {
	if !m.InBounds(c) {
		return true
	}
	return m.blocked[c.Row][c.Col]
}

// SetBlocked 设置栅格占据状态；越界写入被忽略。
func (m *GridMap) SetBlocked(c Cell, v bool) {
	if m.InBounds(c) {
		m.blocked[c.Row][c.Col] = v
	}
}

// WorldToCell 将世界坐标转换为栅格坐标（四舍五入到最近栅格中心）。
// 不保证结果在界内，调用方需用 InBounds / IsBlocked 判断。
func (m *GridMap) WorldToCell(x, y float64) Cell {
	r := int(math.Round((y - m.cfg.OriginY) / m.cfg.Resolution))
	c := int(math.Round((x - m.cfg.OriginX) / m.cfg.Resolution))
	return Cell{Row: r, Col: c}
}

// CellCenter 返回栅格中心的世界坐标。
func (m *GridMap) CellCenter(c Cell) (float64, float64) {
	x := m.cfg.OriginX + float64(c.Col)*m.cfg.Resolution
	y := m.cfg.OriginY + float64(c.Row)*m.cfg.Resolution
	return x, y
}

// Clone 返回地图的深拷贝，用于叠加动态障碍而不污染基础地图。
func (m *GridMap) Clone() *GridMap {
	if m == nil {
		return nil
	}
	cp := NewGridMap(m.cfg)
	for r := range m.blocked {
		copy(cp.blocked[r], m.blocked[r])
	}
	return cp
}

// StampObstacle 将一个轴对齐矩形障碍物（含自身膨胀量）栅格化为占据栅格。
// 障碍物中心或尺寸非法时返回 FailObstacleInvalid 分类错误。
func (m *GridMap) StampObstacle(o Obstacle) error {
	if m == nil {
		return NewError(FailMapMissing, "cannot stamp obstacle on nil map")
	}
	if math.IsNaN(o.X) || math.IsNaN(o.Y) || o.HalfX < 0 || o.HalfY < 0 || o.Inflation < 0 {
		return NewError(FailObstacleInvalid,
			"obstacle %q has invalid geometry: center=(%v,%v) half=(%v,%v) inflation=%v",
			o.ID, o.X, o.Y, o.HalfX, o.HalfY, o.Inflation)
	}
	hw := o.HalfX + o.Inflation
	hh := o.HalfY + o.Inflation
	minC := m.WorldToCell(o.X-hw, o.Y-hh)
	maxC := m.WorldToCell(o.X+hw, o.Y+hh)
	for r := minC.Row; r <= maxC.Row; r++ {
		for c := minC.Col; c <= maxC.Col; c++ {
			cell := Cell{Row: r, Col: c}
			if m.InBounds(cell) {
				m.blocked[r][c] = true
			}
		}
	}
	return nil
}
