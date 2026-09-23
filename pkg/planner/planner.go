package planner

import (
	"container/heap"

	"github.com/lisama569/nav-planner-go/pkg/grid"
)

// defaultMaxExpansions 是默认的节点展开预算（确定性地替代墙上时钟“超时”）。
const defaultMaxExpansions = 1_000_000

// Planner 是有确定展开预算的全局 A* 规划器。
//
// 确定性保证：
//   - 邻域展开顺序固定（grid.Neighbors4 的 上/右/下/左）；
//   - 平局时用 (f, 插入序号) 排序，相同 f 的节点处理顺序与输入一致；
//   - 启发式为曼哈顿距离（四邻域网格下可采纳且一致）；
//   - 因此同一地图、同一对起终点必然产生同一条路径。
type Planner struct {
	maxExpansions int
	expansions    int
}

// New 创建规划器；maxExpansions<=0 表示使用默认预算。
func New(maxExpansions int) *Planner {
	if maxExpansions <= 0 {
		maxExpansions = defaultMaxExpansions
	}
	return &Planner{maxExpansions: maxExpansions}
}

// Plan 在给定地图上搜索从 start 到 goal 的全局路径。
// 返回的路径含起点与终点；失败时 error 为包内预定义的可区分错误。
func (p *Planner) Plan(g *grid.Grid, start, goal grid.Cell) ([]grid.Cell, error) {
	p.expansions = 0
	if g == nil {
		return nil, ErrMapMissing
	}
	if !g.InBounds(start) {
		return nil, ErrStartOutOfBounds
	}
	if !g.InBounds(goal) {
		return nil, ErrGoalOutOfBounds
	}
	if g.Occupied(start) {
		return nil, ErrStartOccupied
	}
	if g.Occupied(goal) {
		return nil, ErrGoalOccupied
	}
	if start == goal {
		return []grid.Cell{start}, nil
	}

	gScore := map[grid.Cell]int{start: 0}
	cameFrom := map[grid.Cell]grid.Cell{}
	closed := map[grid.Cell]bool{}

	open := &pqueue{}
	heap.Init(open)
	insertSeq := 0
	heap.Push(open, &node{cell: start, f: heuristic(start, goal), tie: insertSeq})

	for open.Len() > 0 {
		cur := heap.Pop(open).(*node)
		c := cur.cell
		if closed[c] {
			continue
		}
		closed[c] = true
		p.expansions++
		if p.expansions > p.maxExpansions {
			return nil, ErrPlanLimit
		}
		if c == goal {
			return reconstruct(cameFrom, c), nil
		}
		tentative := gScore[c] + 1
		for _, nb := range g.Neighbors4(c) {
			if g.Occupied(nb) || closed[nb] {
				continue
			}
			if old, ok := gScore[nb]; ok && tentative >= old {
				continue
			}
			gScore[nb] = tentative
			cameFrom[nb] = c
			insertSeq++
			n := &node{cell: nb, f: tentative + heuristic(nb, goal), tie: insertSeq}
			heap.Push(open, n)
		}
	}
	return nil, ErrUnreachable
}

// Expansions 返回上一次搜索实际展开的节点数（用于日志/复现）。
func (p *Planner) Expansions() int { return p.expansions }

// heuristic 是曼哈顿距离（四邻域移动的可采纳启发式）。
func heuristic(a, b grid.Cell) int {
	dx := a.X - b.X
	if dx < 0 {
		dx = -dx
	}
	dy := a.Y - b.Y
	if dy < 0 {
		dy = -dy
	}
	return dx + dy
}

// reconstruct 从终点回溯出含起终点的路径。
func reconstruct(cameFrom map[grid.Cell]grid.Cell, cur grid.Cell) []grid.Cell {
	path := []grid.Cell{cur}
	for {
		prev, ok := cameFrom[cur]
		if !ok {
			break
		}
		path = append(path, prev)
		cur = prev
	}
	// 反转为起点 -> 终点
	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}
	return path
}

// node 是优先队列元素；f 相同则先入队者优先，保证确定性。
type node struct {
	cell grid.Cell
	f    int
	tie  int
	idx  int
}

type pqueue struct {
	pq []*node
}

func (q *pqueue) Len() int { return len(q.pq) }

func (q *pqueue) Less(i, j int) bool {
	if q.pq[i].f != q.pq[j].f {
		return q.pq[i].f < q.pq[j].f
	}
	return q.pq[i].tie < q.pq[j].tie
}

func (q *pqueue) Swap(i, j int) {
	q.pq[i], q.pq[j] = q.pq[j], q.pq[i]
	q.pq[i].idx = i
	q.pq[j].idx = j
}

func (q *pqueue) Push(x any) {
	n := x.(*node)
	n.idx = len(q.pq)
	q.pq = append(q.pq, n)
}

func (q *pqueue) Pop() any {
	n := len(q.pq)
	it := q.pq[n-1]
	q.pq = q.pq[:n-1]
	return it
}
