# clearance — 装配间隙几何内核服务

判断两个**平面凸多边形**是**分离**还是**穿透**，以及在一段**匀速直线运动**里
是否发生接触：

- 单帧（`POST /collide`）：分离时给出最近距离（间隙）、接触法向、最近点对；
  穿透时给出穿透深度（MTVD）、分离方向、接触点。
- 扫掠（`POST /sweep`）：两个零件各带初始位姿与匀速平移速度，在时间窗
  `[0,1]` 内连续判定是否接触；接触时给出**真正的最早接触时刻**、那一刻的
  接触法向与接触点；始终分离时给出**最近接近时刻**、最小间隙与间隙法向。

判定严格走 **Minkowski 差 A⊖B 上的单纯形迭代（GJK）**；含原点时用
**多面体扩展（EPA, Expanding Polytope Algorithm）**求穿透深度与分离方向。
**不使用**包围盒中心距之类的近似，扫掠层也**不做时间离散采样**。

---

## 1. 一条命令起服务

需要 Docker（含 Compose 插件）：

```bash
docker compose up --build
```

服务在 `http://localhost:8080` 暴露查询接口。

不用容器时：

```bash
go run ./cmd/server                 # 默认 :8080
PORT=9000 go run ./cmd/server
```

---

## 2. HTTP 接口

仅 HTTP/JSON，无网页、无交互界面。

### `POST /collide`

请求体：

```json
{
  "polygonA": [{"x": 0, "y": 0}, ...],
  "polygonB": [{"x": 2.75, "y": -0.5}, ...]
}
```

顶点按边界顺序给出（顺时针/逆时针均可），每个多边形至少 3 个顶点。

**分离响应**（`examples/known_gap.json`，两轴对齐矩形、间隙 0.75）：

```bash
curl -s -X POST localhost:8080/collide \
  -H 'Content-Type: application/json' \
  -d @examples/known_gap.json
```

```json
{
  "status": "separated",
  "distance": 0.75,
  "penetrationDepth": 0,
  "normal": {"x": 1, "y": 0},
  "pointA": {"x": 2, "y": 0},
  "pointB": {"x": 2.75, "y": 0},
  "iterations": 2
}
```

字段含义：

| 字段 | 分离 | 穿透 |
| --- | --- | --- |
| `status` | `separated` | `penetrated` |
| `distance` | 最近间隙 | 恒为 0 |
| `penetrationDepth` | 恒为 0 | 穿透深度 |
| `normal` | 由 A 最近点指向 B 最近点的单位法向 | B 应沿之平移以恰好分离的单位方向（A⊖B 最近边的外法向） |
| `pointA` / `pointB` | 最近点对 | 接触点对 |

**穿透响应**（`examples/overlap.json`，x 向重叠 0.3、y 向重叠 0.9，取较小者）：

```bash
curl -s -X POST localhost:8080/collide \
  -H 'Content-Type: application/json' \
  -d @examples/overlap.json
# -> {"status":"penetrated","distance":0,"penetrationDepth":0.3,
#     "normal":{"x":1,...}, ...}
```

**错误响应**（`examples/nonconvex.json` 是凹多边形）：

```json
{ "code": "NON_CONVEX_POLYGON",
  "message": "polygon A: polygon is not convex; only convex polygons are accepted (no convex decomposition)" }
```

错误码：

| HTTP | code | 触发条件 |
| --- | --- | --- |
| 400 | `INVALID_JSON` | 请求体不是合法 JSON / 缺字段 / 坐标非数值 |
| 400 | `TOO_FEW_VERTICES` | 任一多边形顶点数 < 3 |
| 400 | `DEGENERATE_POLYGON` | 面积为 0、共线、连续重复顶点 |
| 400 | `NON_CONVEX_POLYGON` | 凹多边形或自交轮廓（**直接拒绝，不做凸分解**） |
| 400 | `NON_FINITE_COORDINATE` | 坐标为 NaN / ±Inf |
| 422 | `GJK_NO_CONVERGENCE` | 单纯形迭代达到步数上限（128） |
| 422 | `EPA_NO_CONVERGENCE` | 多面体扩展达到步数上限（256） |
| 422 | `EPA_FAILURE` | 无法构造包围原点的多面体 |

迭代上限是硬性保障：用尽即报错，**不会**无限循环，也**不会**把穿透
错报成"距离为零的分离"。边界接触（距离恰好 0）一律归入穿透分支并由
EPA 给出近似为 0 的深度。

另有 `GET /health` 返回 `{"status":"ok"}`。

### `POST /sweep`（连续扫掠，时间维度）

与 `/collide` **完全独立**的入口：老接口的请求/响应格式保持不变。请求在
两个轮廓之外，为每个零件带上**初始位姿偏移**与**匀速平移速度**。零件 X 在
时刻 `t∈[0,1]` 占据 `polygonX + poseX + t·velocityX`。

```bash
curl -s -X POST localhost:8080/sweep \
  -H 'Content-Type: application/json' \
  -d @examples/sweep_contact.json
```

请求字段：

| 字段 | 含义 |
| --- | --- |
| `polygonA` / `polygonB` | 两个凸多边形的本体顶点（同 `/collide` 规则） |
| `poseA` / `poseB` | 可选，初始位姿偏移 `{"x":..,"y":..}`，缺省为 `(0,0)`；`offsetA`/`offsetB` 是同义别名，二者不可同时给 |
| `velocityA` / `velocityB` | 可选，匀速平移速度，缺省为 `(0,0)` |

**会碰响应**（`examples/sweep_contact.json`：A 不动，B 初始边到边间隙 1、以
速度 `(-2,0)` 正对过来，闭式首次接触时刻 = 间隙/接近速率 = 0.5）：

```json
{
  "status": "contact",
  "time": 0.5,
  "distance": 0,
  "penetrationDepth": 0,
  "normal": {"x": 1, "y": 0},
  "pointA": {"x": 1, "y": 0},
  "pointB": {"x": 1, "y": 0},
  "relativeVelocity": {"x": -2, "y": 0},
  "iterations": 46
}
```

**整段安全响应**（`examples/sweep_safe.json`，B 反向背离）：

```json
{
  "status": "safe",
  "time": 0,
  "distance": 1,
  "penetrationDepth": 0,
  "normal": {"x": 1, "y": 0},
  "pointA": {"x": 1, "y": 0},
  "pointB": {"x": 2, "y": 0},
  "relativeVelocity": {"x": 2, "y": 0},
  "iterations": 1
}
```

字段含义：

| 字段 | 会碰 `contact` | 整段安全 `safe` |
| --- | --- | --- |
| `time` | **首次接触时刻** ∈ [0,1] | **最近接近时刻** ∈ [0,1] |
| `distance` | 恒为 0 | 该时间窗内的最小间隙 |
| `penetrationDepth` | 仅"初始即穿透"时非 0（取自 t=0 静态结论） | 恒为 0 |
| `normal` | 接触法向（由 A 接触点指向 B） | 最近时刻的间隙法向 |
| `pointA` / `pointB` | 相切那一刻的接触点（两者重合为一点） | 最近点对 |
| `relativeVelocity` | `velocityB - velocityA`（回显） | 同左 |

**求解方式（不是离散采样）**：把单帧内核当作子过程复用。运动全程的相对
位姿只取决于**相对速度 `w = velocityB − velocityA`**。从当前时刻的最近间隙
`d` 与相对运动沿间隙法向的接近速率 `closing = −w·n` 出发，按凸集距离对平移
的 **1-Lipschitz** 性质做**保守前进**（一步最多 `|w|·dt`，故步长 `d/|w|`
绝不会越过首次接触）；一旦夹住接触，再用**有保护的牛顿/二分 + 两帧割线外推**
把首次归零时刻收敛到容差（割线抵消单帧内核在接触带边缘的恒定间隙偏置，
平滑撞击可达 ~1e-13，擦碰尖点可达 ~1e-9）。

边界语义（均有带数值容差的测试钉住）：

- **初始即穿透**：`time = 0`，接触信息与 t=0 静态穿透结论一致（含深度）。
- **相对速度为零**（同速平移或都不动）：结论与对初始位姿跑一次 `/collide`
  逐字段一致。
- **t=1 恰好贴上**：报接近 1 的接触时刻；**差一点没碰上**：报 `safe`、最近
  时刻 1、给出精确残余间隙。
- **越走越远（背离）**：报 `safe`，最近时刻落在 0。
- **擦碰（相切）**：连续求解能捕捉一闪而过的单点相切；把时间窗均匀切再逐段
  跑静态判定会整段报安全（`examples/sweep_grazing.json`：即使切 10 万段，
  最近采样间隙仍有 1e-5）。

扫掠额外错误码：`NON_FINITE_VELOCITY`（速度分量为 NaN/±Inf，400）、
`SWEEP_NO_CONVERGENCE`（保守前进步数用尽仍未夹住接触或最近点，422；**不会**
把"未收敛"说成"整段安全"）。单帧内核的全部输入校验（顶点不足、退化、非凸、
坐标非有限）在扫掠入口同样生效。

---

## 3. 随附的已知间隙算例

`examples/known_gap.json`：

- A = `[0,2] × [0,1]`
- B = `[2.75,4.25] × [−0.5,1.5]`
- 相对边 x = 2 与 x = 2.75，**精确间隙 = 0.75**
- 接触法向 = **+X**（A 的右边指向 B 的左边）

可直接核对 `distance = 0.75`、`normal = (1,0)`、最近点分别落在两条相对
边上且两点连线长度等于距离。

---

## 4. 算法说明

对凸集 A、B：

1. **支撑函数** `support(d) = h_A(d) − h_B(−d)`（沿方向取投影最大的顶
   点），返回 A⊖B 上的一个支撑点及其在 A、B 上的来源顶点；
2. **GJK 单纯形演化**：从中心差方向起步，在 A⊖B 上构造 1～3 个顶点的
   单纯形，按最近特征（点/Voronoi 面）持续把单纯形向原点推进，并判定原
   点是否在差集内；
3. 原点在差集外 → **分离分支**：原点到最终单纯形最近特征的距离即最近
   间隙，线性插值还原 A、B 上的最近点；
4. 原点在差集内 → **EPA**：从含原点的单纯形出发构造 CCW 凸多边形，反复
   取离原点最近的边、沿其外法向取支撑点并用单调链凸包扩展，直到支撑点
   不再明显越过该边；原点到最近边的距离即穿透深度，外法向即分离方向，
   边两端的来源顶点插值给出接触点。
5. **扫掠层（`sweep.go`，时间维度）**：相对位姿 = 初始位姿 + t·相对速度。
   每一步都调用上面的单帧 `Evaluate` 取当前间隙/法向/最近点；按凸集距离
   对平移的 1-Lipschitz 性质做**保守前进**夹住首次接触，再用二分 + 两帧
   **割线外推**把首次归零时刻收敛到容差。相对运动只依赖两速度之差；这层
   独立成模块，不改动支撑函数、单纯形、分离/EPA 任何一个单帧文件。

容差（在代码中显式定义，测试里同样写明数值）：

- 几何判定相对容差 `1e-10`（按坐标量级缩放，下限 1.0）；
- EPA 扩展收敛阈值 `1e-9`（相对多面体尺度）；
- 扫掠接触间隙 `1e-9`（按场景尺度）、首次接触时刻 `1e-13`、最近时刻
  `1e-12`、接近速率死区 `1e-12`（相对速度量级）；
- 测试中解析值断言容差 `1e-12`，平移一致性断言 `1e-9`（相对）/`1e-9`
  （绝对）。

---

## 5. 代码结构（几何逻辑按职责拆模块）

```
cmd/server/main.go                    服务入口（端口、启动）
internal/
  api/handler.go                      HTTP 接入：DTO、Gin 路由、错误码映射（POST /collide）
  api/sweep_handler.go                扫掠入口：位姿/速度 DTO、POST /sweep（与 /collide 分开）
  geometry/
    vec2.go                           二维向量与容差常量
    polygon.go                        凸性 / 退化 / 顶点数校验
    errors.go                         内核错误码
    support.go                        Minkowski 差支撑函数
    simplex.go                        单纯形演化（线段/三角形）与含原点判定
    gjk.go                            GJK 主迭代（含步数上限、支撑进展终止判据）
    separation.go                     分离分支：最近距离与最近点还原
    epa.go                            穿透分支：多面体扩展（EPA）
    service.go                        单帧对外编排：校验 → GJK → 分离/EPA
    sweep.go                          扫掠层：保守前进 + 割线求根（仅复用单帧 Evaluate）
examples/                             单帧与扫掠已知算例（sweep_*.json）
Dockerfile / docker-compose.yml       容器化与一条命令启动
```

---

## 6. 测试

```bash
go test ./...
go test ./internal/geometry/ -v
```

覆盖（容差均显式写在测试里，非"有返回值"式断言）：

- **非法与退化**：顶点 < 3、共线零面积、连续重复顶点、首尾重复、非数值
  坐标；
- **非凸拒绝**：凹多边形（L 形缺口）与其反向绕序、自交领结；
- **已知间隙轴对齐矩形**：距离精确等于边到边间隔、法向精确为 +X、最近
  点落在相对边上；
- **平移使间隙等量增减**：沿分离法向平移 B，距离变化量等于位移（多组
  位移量、双向）；平移恰好等于间隙时归为穿透接触（深度≈0），不会报
  "距离为零的分离"；
- **整体平移不变性**：两个零件施加同一平移，距离/法向/相对最近点几何
  完全保持；
- **穿透深度对上较小重叠量**：x、y 重叠量不同的多组矩形，穿透深度取较
  小者，法向沿对应轴；沿 MTV 平移恰好变成接触，再多给位移即按该位移量
  分离；
- 随机凸多边形（300 组）与暴力"所有边对最小距离"交叉核对；A/B 互换穿
  透深度一致；迭代步数始终在上限内。

扫掠（`sweep_test.go`、`api/sweep_handler_test.go`）逐条覆盖验收项：

- **闭式首次接触**：轴对齐正方形一静一动，多组接近速率下首次接触时刻精确
  等于 间隙/接近速率（容差 1e-9），接触法向对齐运动轴，接触点落在相对
  边上；
- **背离安全**：同对零件反向运动，报 `safe`、最近时刻恰为 0、最小间隙为
  初始间隙；
- **整体刚性平移/速度不变性（伽利略不变性）**：同时叠加同一初始偏移与同一
  额外速度，首次接触时刻、法向、相对速度不变，世界接触点按
  `偏移 + t·公共速度` 平移（尺度相关容差）；
- **相对速度为零**：多组同速（含共同平移、分离与穿透）扫掠结论与对初始
  位姿跑一次单帧判定在状态/间隙/深度/法向/点上逐字段一致；
- **初始即穿透**：首次接触时刻为 0，深度/法向/点与静态穿透结论一致，与
  后续速度无关；
- **t=1 边界**：恰好 t=1 贴上报接近 1 的接触时刻；差 ε 没碰上报 `safe`、
  最近时刻 1、残余间隙精确为 ε；再多 ε 越过则在 1 之前碰上（闭式时刻）；
- **擦碰不被采样漏掉**：构造顶点–顶点相切路径，均匀切 2…512 段（另用脚本
  验证到 10 万段）逐段跑静态判定全部安全、最近采样间隙远在接触带之上，而
  连续求解在解析时刻 t=1/3 报接触、接触点为该公共顶点；
- **窗内最近接近**：逼近—错过—远离的情形报 `safe` 与解析最近时刻/间隙；
- **位姿偏移生效**、**速度/偏移 NaN/Inf 与非数值被拒**（`NON_FINITE_VELOCITY`
  等）、**步数上限用尽报 `SWEEP_NO_CONVERGENCE`**（不伪报安全）、200 组配置
  始终在预算内；
- 随机凸多边形（120 组）与 4000 点细分网格 + 单元加密求根交叉核对接触判定、
  首次时刻与最小间隙；HTTP 层验证 `/collide` 响应不含任何扫掠字段、行为
  不变。

---

## 7. 构建容器镜像（不用 Compose 时）

```bash
docker build -t clearance-svc:latest .
docker run --rm -p 8080:8080 clearance-svc:latest
```
