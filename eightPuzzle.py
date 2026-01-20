import pygame, time, queue, threading
from collections import deque
import random, heapq, math, itertools

# ============================================================
# FLAT STATE + HELPERS (used by ALL search algorithms)
# ============================================================
GOAL_FLAT = (0, 1, 2, 3, 4, 5, 6, 7, 8)

def to_flat(tiles3x3):
    # tiles3x3 is a tuple-of-tuples or list-of-lists
    return tuple(tiles3x3[i][j] for i in range(3) for j in range(3))

def from_flat(t9):
    # return list-of-lists to be compatible with GUI drawing
    return [list(t9[0:3]), list(t9[3:6]), list(t9[6:9])]

def neighbors(z):
    """
    Generate neighbor blank positions in strict Up, Down, Left, Right order.
    """
    r, c = divmod(z, 3)
    # Up
    if r > 0:     yield z - 3
    # Down
    if r < 2:     yield z + 3
    # Left
    if c > 0:     yield z - 1
    # Right
    if c < 2:     yield z + 1

def swap_blank(state, z, nz):
    lst = list(state)
    lst[z], lst[nz] = lst[nz], lst[z]
    return tuple(lst)

def manhattan(state):
    s = 0
    for idx, v in enumerate(state):
        if v == 0:
            continue
        gi, gj = divmod(v, 3)   # goal row/col for tile v
        ci, cj = divmod(idx, 3) # current row/col
        s += abs(ci - gi) + abs(cj - gj)
    # print(f"manhatten distance: {s}")
    return s

def euclidean(state):
    s = 0.0
    for idx, v in enumerate(state):
        if v == 0:
            continue
        gi, gj = divmod(v, 3)
        ci, cj = divmod(idx, 3)
        s += math.sqrt((ci - gi)**2 +  (cj - gj)**2)
    # print(f"euclidean distance: {s}")
    return s

def reconstruct_path_flat(came_from, goal):
    path = []
    s = goal
    while s is not None:
        path.append(s)
        s = came_from.get(s)
    path.reverse()
    return path

def reconstruct_nodes_with_moves(path_flat):
    """
    Convert a flat path [t0, t1, .., tk] into eight_puzzle nodes with:
      - .tiles (3x3)
      - .prev chain set
      - .move filled as "Up/Down/Left/Right" (None for first)
    """
    nodes = []
    for i, s in enumerate(path_flat):
        node = eight_puzzle(from_flat(s))
        if i > 0:
            prev = path_flat[i - 1]
            z0 = prev.index(0)
            z1 = s.index(0)
            r0, c0 = divmod(z0, 3)
            r1, c1 = divmod(z1, 3)
            if r1 == r0 - 1 and c1 == c0:
                node.move = 'Up'
            elif r1 == r0 + 1 and c1 == c0:
                node.move = 'Down'
            elif c1 == c0 - 1 and r1 == r0:
                node.move = 'Left'
            elif c1 == c0 + 1 and r1 == r0:
                node.move = 'Right'
        nodes.append(node)
    # stitch .prev
    for i in range(1, len(nodes)):
        nodes[i].prev = nodes[i - 1]
    return nodes

# ------------------------------------------------------------
# SECTION 1) PUZZLE STATE + SEARCH ALGORITHMS (GUI wrapper class)
# ------------------------------------------------------------
class eight_puzzle:
    __slots__ = ("tiles", "prev", "move")
    """
    Simple wrapper around a single 8-puzzle state for the GUI/printing:
    - tiles: 3x3 tuple-of-tuples (for drawing)
    - prev:  parent pointer (for reconstructing/animating)
    - move:  move name used to reach this state from its parent
    """
    def __init__(self, tiles):
        self.tiles = tiles if isinstance(tiles, tuple) else tuple(tuple(r) for r in tiles)
        self.prev = None
        self.move = None

    def is_goal(self):
        # Compare with 0..8 row-major
        n = len(self.tiles)
        k = 0
        for i in range(n):
            for j in range(n):
                if self.tiles[i][j] != k:
                    return False
                k += 1
        return True

    # ---------- A* ----------
    def astar(self, heuristic="manhattan"):
        """
        Neighbor order is U, D, L, R (tie-break preserved via heap counter).
        """
        start_time = time.time()
        start = to_flat(self.tiles)

        if start == GOAL_FLAT:
            return [self], 0.0, 0

        # Select heuristic
        h = manhattan if heuristic == "manhattan" else euclidean

        counter = itertools.count()
        frontier = []
        # heap entries: (f, tie_count, g, state)
        heapq.heappush(frontier, (h(start), next(counter), 0, start))
        g_score = {start: 0}
        came_from = {start: None}
        expanded_count = 0
        explored = set()
        max_depth = 0

        while frontier:
            f, _, g, state = heapq.heappop(frontier)

            # Skip if already expanded
            if state in explored:
                continue

            # Check goal
            if state == GOAL_FLAT:
                path_flat = reconstruct_path_flat(came_from, state)
                nodes = reconstruct_nodes_with_moves(path_flat)
                print("Search Depth:", max_depth)
                return nodes, time.time() - start_time, expanded_count

            # Mark as expanded
            explored.add(state)
            expanded_count += 1

            # Expand neighbors
            z = state.index(0)
            for nz in neighbors(z):  # U, D, L, R
                child = swap_blank(state, z, nz)
                new_g = g + 1
                new_f = new_g + h(child)
                # Only consider if not explored or better path found
                if (child not in g_score and child not in explored) or (child in g_score and new_g < g_score[child]) :
                    heapq.heappush(frontier, (new_f, next(counter), new_g, child))
                    g_score[child] = new_g
                    came_from[child] = state
                    max_depth = max(max_depth, new_g)

        # No solution found
        return None, time.time() - start_time, expanded_count


    # ---------- BFS ----------
    def bfs(self):
        start_time = time.time()
        start = to_flat(self.tiles)
        if start == GOAL_FLAT:
            return [self], 0.0, 0

        frontier = deque([start])
        in_frontier = set([start])
        came_from = {start: None}
        explored = {start}
        expanded_count = 0

        while frontier:
            state = frontier.popleft()
            in_frontier.remove(state)

            if state == GOAL_FLAT:
                path_flat = reconstruct_path_flat(came_from, state)
                nodes = reconstruct_nodes_with_moves(path_flat)
                print("Search Depth:", len(nodes) - 1)
                return nodes, time.time() - start_time, expanded_count

            expanded_count += 1

            z = state.index(0)
            for nz in neighbors(z):  # U, D, L, R
                child = swap_blank(state, z, nz)
                if child not in explored and child not in in_frontier:
                    explored.add(child)
                    came_from[child] = state
                    frontier.append(child)
                    in_frontier.add(child)

        return None, time.time() - start_time, expanded_count

    # ---------- DFS ----------
    def dfs(self):
        start_time = time.time()
        start = to_flat(self.tiles)
        if start == GOAL_FLAT:
            return [self], 0.0, 0

        frontier = [(start, 0)]
        in_frontier = set([start])
        explored = set()
        came_from = {start: None}
        expanded_count = 0
        max_depth = 0

        while frontier:
            state, depth = frontier.pop()
            in_frontier.remove(state)

            if state == GOAL_FLAT:
                path_flat = reconstruct_path_flat(came_from, state)
                nodes = reconstruct_nodes_with_moves(path_flat)
                print("Search Depth:", max_depth)
                return nodes, time.time() - start_time, expanded_count

            expanded_count += 1
            explored.add(state)

            z = state.index(0)
            # Push in reverse of U, D, L, R so that the next pop respects U, D, L, R
            nbrs = list(neighbors(z))
            for nz in reversed(nbrs):
                child = swap_blank(state, z, nz)
                if child not in explored and child not in in_frontier:
                    came_from[child] = state
                    frontier.append((child, depth + 1))
                    in_frontier.add(child)
                    max_depth = max(max_depth, depth + 1)

        return None, time.time() - start_time, expanded_count

    def dls(self, start, limit, deadline):
        
        max_depth = 0

        if start == GOAL_FLAT:
            return [start], 0, False, max_depth

        path = [start]
        on_path = {start}

        z = start.index(0)
        # stack frames: (state, zero_pos, depth, neighbor_iter, counted_expansion)
        frontier = [(start, z, 0, iter(neighbors(z)), False)]

        CHECK_MASK = 2048 - 1
        steps = 0
        expanded_count = 0

        while frontier:
            state, zpos, depth, neigh_it, counted = frontier[-1]
            
            # occasional time check
            steps += 1
            if (steps & CHECK_MASK) == 0 and time.time() >= deadline:
                return None, expanded_count, True, max_depth

            # goal?
            if state == GOAL_FLAT:
                return path[:], expanded_count, False, max_depth

            # If we haven't counted this node as "expanded" yet, and it can expand:
            if not counted and depth <= limit :
                expanded_count += 1
                frontier[-1] = (state, zpos, depth, neigh_it, True)

            # expand next neighbor
            try:
                nz = next(neigh_it)
                max_depth = max(max_depth, depth)
            except StopIteration:
                frontier.pop()
                on_path.remove(state)
                path.pop()
                continue

            # generate child
            child = swap_blank(state, zpos, nz)

            # path-cycle block only
            if child in on_path:
                continue

            # cutoff?
            if depth == limit:
                continue

            on_path.add(child)
            path.append(child)

            # push child's frame
            frontier.append((child, nz, depth + 1, iter(neighbors(nz)), False))

        return None, expanded_count, False, max_depth
    
    # ---------- IDS ----------
    def ids(self, depth_limit=31, time_limit=120):
    
        start_time = time.time()
        start_flat = to_flat(self.tiles)
        max_depth = 0
        if start_flat == GOAL_FLAT:
            return [self], 0.0, 0

        deadline = start_time + time_limit
        total_expanded = 0

        for limit in range(depth_limit + 1):
            path_flat, expanded_count, timed_out, current_depth = self.dls(start_flat, limit, deadline)
            total_expanded += expanded_count
            max_depth = max(current_depth, max_depth)

            if timed_out:
                return None, time.time() - start_time, total_expanded

            if path_flat is not None:
                nodes = reconstruct_nodes_with_moves(path_flat)
                print("Search Depth:", max_depth)
                return nodes, time.time() - start_time, total_expanded

        return None, time.time() - start_time, total_expanded


# ------------------------------------------------------------
# SECTION 2) UTILITY HELPERS
# ------------------------------------------------------------

# A function to check if the puzzle provided or generated solvable or not 
def is_solvable(puzzle):
    flat = [num for row in puzzle for num in row if num != 0]
    inv = 0
    for i in range(len(flat)):
        for j in range(i + 1, len(flat)):
            if flat[i] > flat[j]:
                inv += 1
    return inv % 2 == 0

def generate_random_puzzle():
    nums = list(range(9))
    while True:
        random.shuffle(nums)
        p = [nums[i:i + 3] for i in range(0, 9, 3)]
        if is_solvable(p):
            return p

def parse_board_text(text):
    s = text.strip()
    toks = s.split()
    if len(toks) == 9 and all(t.isdigit() for t in toks):
        vals = list(map(int, toks))
        if set(vals) == set(range(9)):
            return [vals[i:i+3] for i in range(0, 9, 3)]
    digits_only = "".join(ch for ch in s if ch.isdigit())
    if len(digits_only) == 9 and set(int(c) for c in digits_only) <= set(range(10)):
        vals = [int(c) for c in digits_only]
        if set(vals) == set(range(9)):
            return [vals[i:i+3] for i in range(0, 9, 3)]
    return None


# ------------------------------------------------------------
# SECTION 3) GUI / THEME / LAYOUT 
# ------------------------------------------------------------
pygame.init()
WIDTH, HEIGHT = 600, 700
win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("8-Puzzle Solver")
try:
    logo = pygame.image.load("8.png")
    pygame.display.set_icon(logo)
except Exception as e:
    print(f"Warning: Could not load icon image. {e}")

font = pygame.font.SysFont("Arial", 34)
small_font = pygame.font.SysFont("Arial", 22)

LIGHT = {
    "bg": (246, 248, 252), "panel": (255, 255, 255), "primary": (78, 133, 255),
    "muted": (230, 234, 242), "text": (25, 28, 33), "text_muted": (88, 96, 105),
    "accent": (59, 201, 130), "danger": (237, 85, 101),
    "tile": (104, 152, 255), "tile_blank": (236, 238, 243), "tile_text": (18, 22, 28),
    "border": (210, 215, 225),
}
DARK = {
    "bg": (18, 20, 24), "panel": (28, 31, 36), "primary": (96, 142, 255),
    "muted": (48, 52, 59), "text": (235, 238, 242), "text_muted": (170, 175, 186),
    "accent": (63, 212, 140), "danger": (243, 109, 124),
    "tile": (106, 146, 255), "tile_blank": (41, 45, 52), "tile_text": (240, 243, 247),
    "border": (55, 60, 68),
}
THEME = LIGHT

RADIUS = 14
BTN_RADIUS = 10
CARD_PADDING = 18
HEADER_Y = 26

HOLD_AT_GOAL_SEC = 2
IDS_TIME_LIMIT_SEC = 120
IDS_MAX_DEPTH = 31

#Initial GUI state setup
puzzle = [[0,1,2],[3,4,5],[6,7,8]]
selected_algo = "bfs"
solution_queue = queue.Queue()
input_mode = False
text_active = False
text_value = ""
solving = False
last_stats = None
toasts = []
dark = False        #boolean for dark mode

CURRENT_LAYOUT = {"board_rect": pygame.Rect(0,0,0,0), "start_x": 0, "start_y": 0, "ts": 0, "gap": 0}

#Creates corner shadows for better visuals 
def draw_shadow(surface, rect, radius=16, spread=8, alpha=70):
    shadow = pygame.Surface((rect.w + spread*2, rect.h + spread*2), pygame.SRCALPHA)
    pygame.draw.rect(shadow, (0, 0, 0, alpha), shadow.get_rect(), border_radius=radius+spread//2)
    shadow = pygame.transform.smoothscale(shadow, (rect.w + spread, rect.h + spread))
    shadow_rect = shadow.get_rect(center=rect.center)
    surface.blit(shadow, shadow_rect)

class Button:
    def __init__(self, rect, text, *, active=False, disabled=False):
        self.rect = rect
        self.text = text
        self.active = active
        self.disabled = disabled
        self.hover = False
    def update_hover(self, mouse_pos):
        self.hover = self.rect.collidepoint(mouse_pos)
    def draw(self, surface):
        base = THEME["muted"] if not self.active else THEME["primary"]
        txt_color = (255,255,255) if self.active else THEME["text"]
        if self.disabled:
            base = (200,205,212); txt_color = (145,150,160)
        lift = -1 if (self.hover and not self.disabled) else 0
        draw_shadow(surface, self.rect.move(0,3+lift), radius=BTN_RADIUS, spread=5, alpha=40)
        pygame.draw.rect(surface, base, self.rect.move(0,lift), border_radius=BTN_RADIUS)
        pygame.draw.rect(surface, THEME["border"], self.rect.move(0,lift), 1, border_radius=BTN_RADIUS)
        label = small_font.render(self.text, True, txt_color)
        surface.blit(label, (self.rect.centerx - label.get_width()/2,
                             self.rect.centery - label.get_height()/2 + lift))
    def clicked(self, event):
        return (event.type == pygame.MOUSEBUTTONDOWN
                and event.button == 1
                and (not self.disabled)
                and self.rect.collidepoint(event.pos))

# Draws a raised panel with shadow and border
def draw_panel(surface, rect):
    draw_shadow(surface, rect, radius=RADIUS, spread=10, alpha=60)
    pygame.draw.rect(surface, THEME["panel"], rect, border_radius=RADIUS)
    pygame.draw.rect(surface, THEME["border"], rect, 1, border_radius=RADIUS)

# If text too wide, scales down but no smaller than 50% of original font size to keep label readable.
def render_centered_clamped_text(surface, text, font_obj, color, y, max_width):
    surf = font_obj.render(text, True, color)
    if surf.get_width() <= max_width:
        surface.blit(surf, (WIDTH//2 - surf.get_width()//2, y))
        return
    scale = max(0.5, min(1.0, max_width / surf.get_width()))
    surf2 = pygame.transform.smoothscale(surf, (int(surf.get_width()*scale), int(surf.get_height()*scale)))
    surface.blit(surf2, (WIDTH//2 - surf2.get_width()//2, y))

# Compute positions for multiple buttons laid out horizontally
def flow_buttons(labels, start_x, start_y, max_width, height=36, gap_x=10, gap_y=10, side_pad=14):
    x, y = start_x, start_y
    out = []
    for lab in labels:
        txt = small_font.render(lab, True, (0, 0, 0))
        w = max(90, txt.get_width() + side_pad*2)
        if x + w > max_width:
            x = start_x
            y += height + gap_y
        out.append((lab, pygame.Rect(x, y, w, height)))
        x += w + gap_x
    return out, y + height

# Computes tile size and starting coordinates
def compute_board_layout(panel_rect):
    inner = panel_rect.inflate(-CARD_PADDING * 2, -CARD_PADDING * 2)
    tile_gap = 10
    ts = (min(inner.w, inner.h) - tile_gap * 2) // 3
    total_w = ts * 3 + tile_gap * 2
    total_h = ts * 3 + tile_gap * 2
    start_x = inner.x + (inner.w - total_w) // 2
    start_y = inner.y + (inner.h - total_h) // 2
    CURRENT_LAYOUT.update({"board_rect": inner, "start_x": start_x, "start_y": start_y, "ts": ts, "gap": tile_gap})

# Return a rectangle for tile at row i, col j
def tile_rect(i, j):
    ts = CURRENT_LAYOUT["ts"]; gap = CURRENT_LAYOUT["gap"]
    x = CURRENT_LAYOUT["start_x"] + j*(ts + gap)
    y = CURRENT_LAYOUT["start_y"] + i*(ts + gap)
    return pygame.Rect(x, y, ts, ts)

# Draws the 3x3 tiles
def draw_puzzle_tiles(state, offsets=None):
    for i in range(3):
        for j in range(3):
            val = state[i][j]
            r = tile_rect(i, j)
            if offsets and (i, j) in offsets:
                ox, oy = offsets[(i, j)]
                r = r.move(ox, oy)
            draw_shadow(win, r, radius=12, spread=6, alpha=35)
            color = THEME["tile"] if val != 0 else THEME["tile_blank"]
            pygame.draw.rect(win, color, r, border_radius=12)
            pygame.draw.rect(win, THEME["border"], r, 1, border_radius=12)
            if val != 0:
                text_surf = font.render(str(val), True, THEME["tile_text"])
                win.blit(text_surf, (r.centerx - text_surf.get_width()/2, r.centery - text_surf.get_height()/2 - 2))

# Draws and prints the msgs that appear on top on the window
def draw_toasts():
    now = time.time()
    live = []
    y = 8
    for msg, color, t_exp in toasts:
        if now < t_exp:
            surf = small_font.render(msg, True, (255,255,255))
            box = pygame.Rect(WIDTH//2 - (surf.get_width()+24)//2, y, surf.get_width()+24, 32)
            draw_shadow(win, box, radius=10, spread=6, alpha=60)
            pygame.draw.rect(win, color, box, border_radius=10)
            win.blit(surf, (box.x+12, box.y+6))
            y += 40
            live.append((msg, color, t_exp))
    toasts[:] = live

def add_toast(msg, kind="info", dur=2.2):
    color = {"info": THEME["primary"], "ok": THEME["accent"], "err": THEME["danger"]}.get(kind, THEME["primary"])
    toasts.append((msg, color, time.time() + dur))

# ------------------------------------------------------------
# SECTION 4) MANUAL PLAY HELPERS
# ------------------------------------------------------------
def find_blank(grid):
    for i in range(3):
        for j in range(3):
            if grid[i][j] == 0:
                return i, j
    return None, None

def slide_blank_animated(grid, di, dj):
    bi, bj = find_blank(grid)
    if bi is None:
        return False
    ni, nj = bi + di, bj + dj
    if not (0 <= ni < 3 and 0 <= nj < 3):
        return False

    steps = 8
    dx = dj * (CURRENT_LAYOUT["ts"] + CURRENT_LAYOUT["gap"]) // steps
    dy = di * (CURRENT_LAYOUT["ts"] + CURRENT_LAYOUT["gap"]) // steps
    offsets = {}
    for s in range(1, steps + 1):
        offsets[(bi, bj)] = (dx * s, dy * s)
        offsets[(ni, nj)] = (-dx * s, -dy * s)
        redraw_all(offsets); pygame.display.update(); pygame.event.pump()
        time.sleep(0.012)

    grid[bi][bj], grid[ni][nj] = grid[ni][nj], grid[bi][bj]
    return True

# ------------------------------------------------------------
# SECTION 5) MAIN SCREEN RENDER
# ------------------------------------------------------------
def redraw_all(offsets=None):
    global dark
    win.fill(THEME["bg"])

    title = font.render("8-Puzzle Solver", True, THEME["text"])
    win.blit(title, (WIDTH//2 - title.get_width()//2, HEADER_Y + 15))
    if last_stats:
        s = f"{last_stats['algo']} • Moves {last_stats['moves']} • Expanded {last_stats['expanded']} • {last_stats['time']:.2f}s"
        render_centered_clamped_text(win, s, small_font, THEME["text_muted"], HEADER_Y + 60, max_width=WIDTH - 40)

    algo_labels = ["BFS", "DFS", "IDS", "A* (Manhattan)", "A* (euclidean)"]
    prelim, _ = flow_buttons(algo_labels, 0, 0, WIDTH)
    total_w = sum(r.width for _, r in prelim) + (len(prelim) - 1) * 10
    start_x = max(75, (WIDTH - total_w) // 2)
    algo_rects, selector_bottom = flow_buttons(algo_labels, start_x, 120, WIDTH - 20)

    buttons_top = []
    for label, rect in algo_rects:
        b = Button(rect, label, active=(selected_algo.lower() == label.lower()))
        b.update_hover(pygame.mouse.get_pos()); b.draw(win)
        buttons_top.append((label, b))

    panel_margin = 20
    panel_width  = max(420, WIDTH - 160)
    panel_height = max(260, HEIGHT - (selector_bottom + 180))
    panel_x = (WIDTH - panel_width) // 2
    panel_y = selector_bottom + panel_margin
    panel_rect = pygame.Rect(panel_x, panel_y, panel_width, panel_height)
    draw_panel(win, panel_rect)
    compute_board_layout(panel_rect)
    draw_puzzle_tiles(puzzle, offsets)

    if input_mode:
        hint = "Manual: Use arrow keys to move. Press T to type, Esc to exit."
        hint_surf = small_font.render(hint, True, THEME["text_muted"])
        win.blit(hint_surf, (panel_rect.centerx - hint_surf.get_width()//2,
                             panel_rect.bottom - CARD_PADDING + 25))
        if text_active:
            sub = "Enter board as: 1 2 3 4 5 6 7 8 0  OR  123 567 780"
            sub_surf = small_font.render(sub, True, THEME["text_muted"])
            win.blit(sub_surf, (panel_rect.centerx - sub_surf.get_width()//2,
                                panel_rect.bottom - CARD_PADDING + 50))
            box_w = min(panel_rect.w - CARD_PADDING*2, 520)
            box = pygame.Rect(0, 0, box_w, 30)
            box.centerx = panel_rect.centerx
            box.bottom  = panel_rect.bottom - CARD_PADDING + 110
            pygame.draw.rect(win, (245, 245, 245) if not dark else (55, 60, 68), box, border_radius=8)
            pygame.draw.rect(win, THEME["border"], box, 1, border_radius=8)
            placeholder = "e.g. 123 567 780  • Press Enter"
            shown = text_value if text_value else placeholder
            color = (255,255,255) if dark else ( THEME["text"] if text_value else THEME["text_muted"] )
            if not text_value and not dark:
                color = THEME["text_muted"]
            surf = small_font.render(shown, True, color)
            win.blit(surf, (box.x + 10, box.y + 2))

    labels_bottom = ["Quit", "Manual", "Random", "Solve", "Restart"]
    buttons_bottom = {}
    total_bottom_w = len(labels_bottom) * 100 + (len(labels_bottom) - 1) * 10
    bx = (WIDTH - total_bottom_w) // 2
    by = HEIGHT - 60
    for i, lbl in enumerate(labels_bottom):
        active = (lbl == "Manual" and (input_mode or text_active)) or (lbl == "Solve" and solving)
        disabled = (lbl == "Solve" and solving)
        rect = pygame.Rect(bx + i * 110, by, 100, 38)
        buttons_bottom[lbl] = Button(rect, lbl, active=active, disabled=disabled)
        buttons_bottom[lbl].update_hover(pygame.mouse.get_pos()); buttons_bottom[lbl].draw(win)

    draw_toasts()
    return buttons_top, buttons_bottom

# ------------------------------------------------------------
# SECTION 6) CONSOLE OUTPUT HELPERS
# ------------------------------------------------------------
def print_move_sequence_limited(path, limit_moves=50):
    seq = []
    steps = min(limit_moves + 1, len(path))  # +1 states for N moves
    for i in range(steps):
        move = "Start" if path[i].move is None else path[i].move
        seq.append(move)
    print(f"Move sequence (first {min(limit_moves, len(path)-1)} moves):", " -> ".join(seq))

# ------------------------------------------------------------
# SECTION 7) SOLVER THREAD 
# ------------------------------------------------------------
def solve_puzzle():
    global solving, last_stats
    solving = True
    start = eight_puzzle(puzzle)

    print(f"\n=== Running {selected_algo.upper()} ===")
    if selected_algo == "bfs":
        path, t, expanded = start.bfs()
    elif selected_algo == "dfs":
        path, t, expanded = start.dfs()
    elif selected_algo == "ids":
        path, t, expanded = start.ids(depth_limit=IDS_MAX_DEPTH, time_limit=IDS_TIME_LIMIT_SEC)
    elif selected_algo == "a* (manhattan)":
        path, t, expanded = start.astar(heuristic="manhattan")
    elif selected_algo == "a* (euclidean)":
        path, t, expanded = start.astar(heuristic="euclidean")
    else:
        solving = False
        return

    moves = (len(path) - 1) if path else 0
    print(f"Algorithm: {selected_algo.upper()}")
    print(f"Moves: {moves}")
    print(f"Cost of Path: {moves}")
    print(f"Expanded Nodes: {expanded}")
    print(f"Execution Time: {t:.2f}s")
    last_stats = {"algo": selected_algo.upper(), "moves": moves, "expanded": expanded, "time": t}

    if path:
        if selected_algo == "dfs" and len(path) > 51:
            print_move_sequence_limited(path, limit_moves=50)
        else:
            print_move_sequence_limited(path, limit_moves=max(0, len(path)-1))

    if selected_algo == "ids" and path is None:
        add_toast("IDS timed out (2 min limit)", "err")

    solution_queue.put((selected_algo.lower(), path if path else []))
    solving = False

def reset_puzzle():
    global puzzle, input_mode, text_active, text_value, last_stats
    puzzle = [[0,1,2],[3,4,5],[6,7,8]]
    input_mode = False
    text_active = False
    text_value = ""
    last_stats = None
    add_toast("Reset", "info", 1.5)

# ------------------------------------------------------------
# SECTION 8) MAIN LOOP
# ------------------------------------------------------------
running = True
clock = pygame.time.Clock()

while running:
    # throttle FPS while solving (reduces GUI contention)
    clock.tick(5 if solving else 60)

    if not solving:
        buttons_top, buttons_bottom = redraw_all(offsets=None)
        pygame.display.update()
    else:
        pygame.event.pump()
        time.sleep(0.02)
        buttons_top, buttons_bottom = [], {}

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Theme toggle (D)
        if event.type == pygame.KEYDOWN and event.key == pygame.K_d:
            dark = not dark
            THEME = DARK if dark else LIGHT

        # Manual mode: arrows (unless typing)
        if event.type == pygame.KEYDOWN and input_mode and not text_active:
            moved = False
            if event.key == pygame.K_UP:      moved = slide_blank_animated(puzzle, -1, 0)
            elif event.key == pygame.K_DOWN:  moved = slide_blank_animated(puzzle,  1, 0)
            elif event.key == pygame.K_LEFT:  moved = slide_blank_animated(puzzle,  0,-1)
            elif event.key == pygame.K_RIGHT: moved = slide_blank_animated(puzzle,  0, 1)
            elif event.key == pygame.K_t:
                text_active = True; text_value = ""
            elif event.key == pygame.K_ESCAPE:
                input_mode = False; text_active = False; text_value = ""
            if moved: last_stats = None

        # Manual mode: typing board + Enter to set
        if event.type == pygame.KEYDOWN and text_active:
            if event.key == pygame.K_RETURN:
                candidate = parse_board_text(text_value)
                if candidate and is_solvable(candidate):
                    puzzle[:] = candidate
                    last_stats = None
                    add_toast("Board updated", "ok")
                elif candidate:
                    add_toast("Not solvable", "err")
                else:
                    add_toast("Invalid input", "err")
                text_active = False; text_value = ""
            elif event.key == pygame.K_ESCAPE:
                text_active = False; text_value = ""
            elif event.key == pygame.K_BACKSPACE:
                text_value = text_value[:-1]
            else:
                if event.unicode.isdigit() or event.unicode == ' ':
                    text_value += event.unicode

        # Top buttons
        for label, btn in buttons_top:
            if btn.clicked(event):
                selected_algo = label.lower()

        # Bottom bar (guard if we skipped drawing)
        if "Quit" in buttons_bottom and buttons_bottom["Quit"].clicked(event):
            running = False
        if "Manual" in buttons_bottom and buttons_bottom["Manual"].clicked(event):
            input_mode = True; text_active = False; text_value = ""
        if "Random" in buttons_bottom and buttons_bottom["Random"].clicked(event):
            puzzle = generate_random_puzzle(); last_stats = None; add_toast("Randomized", "info")
        if "Restart" in buttons_bottom and buttons_bottom["Restart"].clicked(event):
            reset_puzzle()
        if "Solve" in buttons_bottom and buttons_bottom["Solve"].clicked(event):
            if not solving:
                threading.Thread(target=solve_puzzle, daemon=True).start()

        # Click-to-slide (adjacent tiles)
        if event.type == pygame.MOUSEBUTTONDOWN and input_mode and not text_active and event.button == 1:
            mx, my = event.pos
            br = CURRENT_LAYOUT["board_rect"]
            if br.collidepoint(mx, my) and CURRENT_LAYOUT["ts"] > 0:
                j = int((mx - CURRENT_LAYOUT["start_x"]) // (CURRENT_LAYOUT["ts"] + CURRENT_LAYOUT["gap"]))
                i = int((my - CURRENT_LAYOUT["start_y"]) // (CURRENT_LAYOUT["ts"] + CURRENT_LAYOUT["gap"]))
                bi, bj = find_blank(puzzle)
                if bi is not None and 0 <= i < 3 and 0 <= j < 3 and (abs(i - bi) + abs(j - bj) == 1):
                    di, dj = i - bi, j - bj
                    if slide_blank_animated(puzzle, di, dj):
                        last_stats = None

    # Animate solution if queued (cap DFS at first 50 moves)
    if not solution_queue.empty():
        algo, path = solution_queue.get()
        if path:
            original_puzzle = [row[:] for row in puzzle]
            max_states = len(path)
            if algo == "dfs" and len(path) > 51:
                max_states = 51  # first 50 moves
                add_toast("DFS animation: showing first 50 moves", "info")

            for st in path[:max_states]:
                win.fill(THEME["bg"])
                puzzle = [list(r) for r in st.tiles]
                redraw_all(offsets=None)
                pygame.display.update()
                pygame.event.pump()
                time.sleep(0.25)

            pygame.event.pump()
            time.sleep(HOLD_AT_GOAL_SEC)
            puzzle = original_puzzle
            add_toast("Animation done", "ok", 1.5)

pygame.quit()
