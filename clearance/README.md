# clearance — 装配间隙几何内核服务

判断两个**平面凸多边形**是**分离**还是**穿透**：

- 分离时：给出最近距离（间隙）、接触法向、两零件上的最近点对；
- 穿透时：给出穿透深度（最小平移距离 MTVD）、把两者恰好推开的分离方向、接触点。

判定严格走 **Minkowski 差 A⊖B 上的单纯形迭代（GJK）**；含原点时用
**多面体扩展（EPA, Expanding Polytope Algorithm）**求穿透深度与分离方向。
**不使用**包围盒中心距之类的近似。

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
- 测试中解析值断言容差 `1e-12`，平移一致性断言 `1e-9`（相对）/`1e-9`
  （绝对）。

---

## 5. 代码结构（几何逻辑按职责拆模块）

```
cmd/server/main.go                    服务入口（端口、启动）
internal/
  api/handler.go                      HTTP 接入：DTO、Gin 路由、错误码映射
  geometry/
    vec2.go                           二维向量与容差常量
    polygon.go                        凸性 / 退化 / 顶点数校验
    errors.go                         内核错误码
    support.go                        Minkowski 差支撑函数
    simplex.go                        单纯形演化（线段/三角形）与含原点判定
    gjk.go                            GJK 主迭代（含步数上限）
    separation.go                     分离分支：最近距离与最近点还原
    epa.go                            穿透分支：多面体扩展（EPA）
    service.go                        对外编排：校验 → GJK → 分离/EPA
examples/                             已知间隙、穿透、非法输入算例
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

---

## 7. 构建容器镜像（不用 Compose 时）

```bash
docker build -t clearance-svc:latest .
docker run --rm -p 8080:8080 clearance-svc:latest
```
