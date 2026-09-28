# clearance — 装配间隙几何内核服务

判断两个**平面凸多边形**是**分离**还是**穿透**：

- 分离时：给出最近距离（间隙）、接触法向、两零件上的最近点对；
- 穿透时：给出穿透深度（最小平移距离 MTVD）、把两者恰好推开的分离方向、接触点。

判定严格走 **Minkowski 差 A⊖B 上的单纯形迭代（GJK）**；含原点时用
**多面体扩展（EPA, Expanding Polytope Algorithm）**求穿透深度与分离方向。
**不使用**包围盒中心距之类的近似。

除单帧静态查询 `POST /collide` 外，服务还提供**扫掠查询** `POST /sweep`：
两个零件在时间窗 `[0,1]` 内各自匀速平移，判定这段**运动过程**中是否接触，
给出真实首次接触时刻（连续收敛求解，不做时间窗离散采样），或整段安全时
的最近接近时刻、最小间隙与间隙法向。

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

### `POST /sweep`（扫掠：把时间维算进来）

请求体：

```json
{
  "polygonA":  [{"x": 0, "y": 0}, ...],
  "polygonB":  [{"x": 0, "y": 0}, ...],
  "offsetA":   {"x": 0, "y": 0},
  "offsetB":   {"x": 1.5, "y": 0},
  "velocityA": {"x": 0, "y": 0},
  "velocityB": {"x": -2, "y": 0}
}
```

- 顶点是零件**局部系**下的凸多边形；`offsetA/offsetB` 是局部系在 `t=0`
  的初始世界偏移（可省略，默认零向量）；
- `velocityA/velocityB` 是时间窗 `t ∈ [0,1]` 内的**匀速平移速度**，两个
  字段必填且必须是数值；
- 零件世界位姿随时间线性推进：`P_k(t) = polygon_k + offset_k + velocity_k·t`。
  相对运动只取决于 `velocityB - velocityA`。

**会碰响应**（`examples/sweep_contact.json`：初始间隙 0.5、正对着以相对
速率 2 接近，闭式 TOI = 0.5/2 = 0.25）：

```bash
curl -s -X POST localhost:8080/sweep \
  -H 'Content-Type: application/json' \
  -d @examples/sweep_contact.json
```

```json
{
  "status": "contact",
  "time": 0.25,
  "distance": 0,
  "penetrationDepth": 0,
  "normal": {"x": 1, "y": 0},
  "pointA": {"x": 1, "y": 0},
  "pointB": {"x": 1, "y": 0},
  "iterations": 33
}
```

**整段安全响应**（`examples/sweep_safe.json`：两者背离，最近时刻落在
`t=0`）：

```json
{
  "status": "safe",
  "time": 0,
  "distance": 0.5,
  "penetrationDepth": 0,
  "normal": {"x": 1, "y": 0},
  "pointA": {"x": 1, "y": 0},
  "pointB": {"x": 1.5, "y": 0},
  "iterations": 1
}
```

| 字段 | 会碰（`contact`） | 整段安全（`safe`） |
| --- | --- | --- |
| `time` | 首次接触时刻（TOI，∈ [0,1]） | 最小间隙对应的最近时刻（∈ [0,1]） |
| `distance` | 0 | 该时刻的最小间隙 |
| `penetrationDepth` | 0（首次触碰）；仅初始即穿透时非 0 | 恒为 0 |
| `normal` | 相切时刻由 A 指向 B 的接触法向 | 最近时刻由 A 指向 B 的间隙法向 |
| `pointA` / `pointB` | 相切点对（世界坐标） | 最近点对（世界坐标） |
| `iterations` | 消耗的单帧内核评估次数 | 同左 |

边界语义：

- **初始即穿透**：`time = 0`，接触信息取初始位姿的静态穿透结论（即使
  速度本可在之后把两者拉开）；
- **恰在 `t=1` 贴上**：报接近 1 的接触时刻；`t=1` 仍差一点没碰上（且仍
  在接近）：报整段安全、最近时刻为 1；
- **背离运动**：整段安全，最近时刻为 0；
- **相对速度为零**（速度相等，包括都不动）：扫掠结论与对初始位姿跑一次
  `/collide` 完全一致（间隙/穿透、法向、点对）。

扫掠新增错误码：

| HTTP | code | 触发条件 |
| --- | --- | --- |
| 400 | `NON_FINITE_VELOCITY` | 速度分量为 NaN / ±Inf，或缺速度字段 |
| 422 | `SWEEP_NO_CONVERGENCE` | 保守前进迭代达到步数上限（64）仍未收敛到接触容差 |

其余非法输入（顶点不足、退化、非凸、坐标非有限）沿用 `/collide` 的错误
码与 400 状态。

求解方法（不是离散采样）：在任一固定时刻复用单帧内核得到间隙 `g` 与间隙
法向 `n`，间隙的右导数为 `(vB−vA)·n`；按 `Δt = g / −(vB−vA)·n` 做**保守
前进**（凸间隙函数的支撑线保证该步内不可能穿透），再对首次过零括号做
二分，收敛到真实首次接触时刻；间隙由减转增（错过）时同样用二分定位最近
时刻。**不会**把时间窗切成若干段逐段静态判定。

另有 `GET /health` 返回 `{"status":"ok"}`。

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

容差（在代码中显式定义，测试里同样写明数值）：

- 几何判定相对容差 `1e-10`（按坐标量级缩放，下限 1.0）；
- EPA 扩展收敛阈值 `1e-9`（相对多面体尺度）；
- 扫掠接触容差 `1e-10`（仅按坐标量级缩放，不按速度缩放）；首次接触
  时刻的时间括号二分至 `1e-13`；扫掠步数上限 64，二分上限各 64；
- 测试中解析值断言容差 `1e-12`，平移一致性断言 `1e-9`（相对）/`1e-9`
  （绝对）；扫掠解析 TOI 断言容差 `1e-9`（单帧内核按坐标量级给出的
  间隙分辨地板除以接近速率）。

---

## 5. 代码结构（几何逻辑按职责拆模块）

```
cmd/server/main.go                    服务入口（端口、启动）
internal/
  api/handler.go                      /collide HTTP 接入：DTO、Gin 路由、错误码映射
  api/sweep_handler.go                /sweep HTTP 接入：位姿/速度 DTO 与校验
  geometry/
    vec2.go                           二维向量与容差常量
    polygon.go                        凸性 / 退化 / 顶点数校验
    errors.go                         内核错误码
    support.go                        Minkowski 差支撑函数
    simplex.go                        单纯形演化（线段/三角形）与含原点判定
    gjk.go                            GJK 主迭代（含步数上限）
    separation.go                     分离分支：最近距离与最近点还原
    epa.go                            穿透分支：多面体扩展（EPA）
    service.go                        对外编排：校验 → GJK → 分离/EPA（单帧）
    motion.go                         匀速运动模型、扫掠校验与扫掠结果编排
    sweep.go                          扫掠层：保守前进 + 首次过零/最近时刻二分
                                      （唯一几何原语是单帧 Evaluate，不另写几何）
  api/handler.go                      /collide 接入：DTO、路由、错误码映射
  api/sweep_handler.go                /sweep 接入：位姿/速度 DTO 与校验
examples/                             已知间隙、穿透、非法输入、扫掠算例
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

扫掠查询（`sweep_test.go` / `sweep_handler_test.go`）逐条覆盖：

- **闭式 TOI**：轴对齐正方形正对平移，首次接触时刻精确等于
  间隙/接近速率（多组，含 y 轴、双向同时运动、斜向三角形）；接触法向
  对上运动轴/斜边法向，接触点对在 TOI 重合；
- **背离**：同对正方形改成背离速度，报整段安全、最近时刻为 0；
- **整体平移/整体速度不变性**：两者叠加同一初始偏移与同一额外速度，
  TOI 与法向不变，接触点仅相差公共位移；
- **相对速度为零**：扫掠结论与对初始位姿的一次静态判定逐项一致
  （分离/穿透、间隙或深度、法向、点对）；
- **初始即穿透**：TOI = 0，接触信息与静态穿透一致；
- **t=1 边界**：恰在末端贴上报接近 1 的接触时刻；末端仍差固定
  shortfall 没碰上报安全、最近时刻为 1、残差间隙等于 shortfall；
  保守步越过末端但末端已穿透时，真实 TOI 在 t<1 仍被二分捕获；
- **防离散采样擦碰**：接触区间 (0.26,0.30) 不含任何 k/8 网格点，逐段
  静态采样会整段漏报，连续求解给出接触与解析 TOI 0.26；
- **擦边错过**：内部最近时刻、最小间隙、法向对独立解析角点模型与
  2000 点稠密扫描核对；
- 200 组随机凸多边形随机运动与 4001 点独立稠密扫描核对
  会碰/安全、TOI 括号、最小间隙与最近时刻；
- 步数上限强制耗尽时报 `SWEEP_NO_CONVERGENCE`，不会硬说成整段安全；
- 非法输入：顶点不足、退化、非凸、坐标非有限、速度 NaN/±Inf、偏移
  非有限均被拒绝并带可读错误码；
- HTTP 层验证会碰/安全/带偏移的整体运动/各类错误码，并验证老的
  `/collide` 请求与响应格式保持不变、不含任何扫掠字段。

---

## 7. 构建容器镜像（不用 Compose 时）

```bash
docker build -t clearance-svc:latest .
docker run --rm -p 8080:8080 clearance-svc:latest
```
