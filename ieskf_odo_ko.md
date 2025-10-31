# FAST-LIO2용 IMU 경사 보정 오도미터 결합 IESKF

## 0. 표기

- 좌표계: 월드(w), 바디/IMU(b), 라이다(ℓ)
- 회전행렬: \( \mathbf{R} \equiv \mathbf{R}_{b}^{w} \), \( \mathbf{R}_{\ell i} \equiv \mathbf{R}_{\ell}^{b} \)
- \( [\cdot]_\times \): 외적 행렬, \( \exp(\cdot) \): SO(3) 지수맵

## 1. 상태·오차상태

상태
$$
\mathbf{x}=
\{\ \mathbf{p}\in\mathbb{R}^3,\ 
\mathbf{R}\in SO(3),\ 
\mathbf{R}_{\ell i}\in SO(3),\ 
\mathbf{t}_{\ell i}\in\mathbb{R}^3,\ 
\mathbf{v}\in\mathbb{R}^3,\ 
\mathbf{b}_g\in\mathbb{R}^3,\ 
\mathbf{b}_a\in\mathbb{R}^3,\ 
\hat{\mathbf{g}}\in\mathbb{S}^2,\ 
s_v\in\mathbb{R}\ \}
$$

오차상태
$$
\delta\mathbf{x}=
\{\ \delta\mathbf{p},\ \delta\boldsymbol{\theta},\ 
\delta\boldsymbol{\theta}_{\ell i},\ \delta\mathbf{t}_{\ell i},\ 
\delta\mathbf{v},\ \delta\mathbf{b}_g,\ \delta\mathbf{b}_a,\ 
\delta\boldsymbol{\xi}_g,\ \delta s_v\ \}
$$

리트랙션
$$
\mathbf{R}\leftarrow \mathbf{R}\exp([\delta\boldsymbol{\theta}]_\times),\quad
\mathbf{R}_{\ell i}\leftarrow \mathbf{R}_{\ell i}\exp([\delta\boldsymbol{\theta}_{\ell i}]_\times)
$$

$$
\mathbf{p}\leftarrow \mathbf{p}+\delta\mathbf{p},\ \ 
\mathbf{t}_{\ell i}\leftarrow \mathbf{t}_{\ell i}+\delta\mathbf{t}_{\ell i},\ \ 
\mathbf{v}\leftarrow \mathbf{v}+\delta\mathbf{v},\ \ 
\mathbf{b}_g\leftarrow \mathbf{b}_g+\delta\mathbf{b}_g,\ \ 
\mathbf{b}_a\leftarrow \mathbf{b}_a+\delta\mathbf{b}_a,\ \
s_v\leftarrow s_v+\delta s_v
$$

$$
\hat{\mathbf{g}}\leftarrow \exp_{\mathbb{S}^2}(\delta\boldsymbol{\xi}_g)\ \hat{\mathbf{g}}
$$

## 2. IMU 전파

바이어스 보정
$$
\mathbf{a}=\mathbf{a}_m-\mathbf{b}_a,\qquad
\boldsymbol{\omega}=\boldsymbol{\omega}_m-\mathbf{b}_g
$$

이산 전파(\(\Delta t\))
$$
\mathbf{R}_{k+1}=\mathbf{R}_k\exp([\boldsymbol{\omega}_k]_\times\Delta t)
$$
$$
\mathbf{v}_{k+1}=\mathbf{v}_k+\big(\mathbf{R}_k\mathbf{a}_k+g_0\,\hat{\mathbf{g}}_k\big)\Delta t
$$
$$
\mathbf{p}_{k+1}=\mathbf{p}_k+\mathbf{v}_k\Delta t+\\tfrac{1}{2}\big(\mathbf{R}_k\mathbf{a}_k+g_0\,\hat{\mathbf{g}}_k\big)\Delta t^2
$$

랜덤워크
$$
\mathbf{b}_{g,k+1}=\mathbf{b}_{g,k}+\mathbf{n}_{bg}\Delta t,\quad
\mathbf{b}_{a,k+1}=\mathbf{b}_{a,k}+\mathbf{n}_{ba}\Delta t
$$
$$
\hat{\mathbf{g}}_{k+1}=\exp_{\mathbb{S}^2}(\mathbf{n}_{gd}\Delta t)\ \hat{\mathbf{g}}_k,\quad
s_{v,k+1}=s_{v,k}+n_{sv}\Delta t
$$

공분산 전파
$$
\mathbf{P}_{k+1}=\boldsymbol{\Phi}_k\,\mathbf{P}_k\,\boldsymbol{\Phi}_k^\top+\mathbf{Q}_k
$$

## 3. LiDAR 업데이트(점-평면, IESKF)

포인트 \(i\)에 대해
$$
r_i^{(L)}=
\mathbf{n}_i^\top\!\left(
\mathbf{R}\big(\mathbf{R}_{\ell i}\mathbf{p}_i^\ell+\mathbf{t}_{\ell i}\big)
+\mathbf{p}-\mathbf{q}_i\right)
$$

자코비안
$$
\frac{\partial r_i^{(L)}}{\partial\,\delta\mathbf{p}}=\mathbf{n}_i^\top,\qquad
\frac{\partial r_i^{(L)}}{\partial\,\delta\boldsymbol{\theta}}
=-\mathbf{n}_i^\top\,\mathbf{R}\,[\mathbf{q}_i]_\times
$$
$$
\frac{\partial r_i^{(L)}}{\partial\,\delta\boldsymbol{\theta}_{\ell i}}
=-\mathbf{n}_i^\top\,\mathbf{R}\,\mathbf{R}_{\ell i}\,[\mathbf{p}_i^\ell]_\times,\qquad
\frac{\partial r_i^{(L)}}{\partial\,\delta\mathbf{t}_{\ell i}}=\mathbf{n}_i^\top\,\mathbf{R}
$$

## 4. 오도미터 업데이트(경사 보정)

### 4.1 지면 접선 방향
$$
\mathbf{v}_\perp=(\mathbf{I}-\hat{\mathbf{g}}\hat{\mathbf{g}}^\top)\,\mathbf{v},\qquad
\mathbf{u}_w=\frac{\mathbf{v}_\perp}{\|\mathbf{v}_\perp\|},\qquad
\mathbf{u}_b=\mathbf{R}^\top\mathbf{u}_w
$$

### 4.2 속도 잔차(방향형)
$$
\mathbf{r}^{(v)}=
s_v\,v_{\mathrm{odo}}\ \mathbf{u}_b\ -\ \mathbf{R}^\top\mathbf{v}
$$

### 4.3 선택적 크기-전용 잔차
$$
r_{\mathrm{mag}}=\|\mathbf{R}^\top\mathbf{v}\|-s_v\,v_{\mathrm{odo}}
$$

### 4.4 자코비안
$$
\frac{\partial \mathbf{r}^{(v)}}{\partial\,\delta\mathbf{p}}=\mathbf{0}_{3\times3},\qquad
\frac{\partial \mathbf{r}^{(v)}}{\partial\,\delta\boldsymbol{\theta}}=(\mathbf{R}^\top\mathbf{v})_\times,\qquad
\frac{\partial \mathbf{r}^{(v)}}{\partial\,\delta\mathbf{v}}=-\mathbf{R}^\top
$$
$$
\frac{\partial \mathbf{r}^{(v)}}{\partial\,\delta\mathbf{b}_g}=\mathbf{0},\quad
\frac{\partial \mathbf{r}^{(v)}}{\partial\,\delta\mathbf{b}_a}=\mathbf{0},\quad
\frac{\partial \mathbf{r}^{(v)}}{\partial\,\delta\boldsymbol{\xi}_g}=\mathbf{0}_{3\times2},\quad
\frac{\partial \mathbf{r}^{(v)}}{\partial\,\delta s_v}=v_{\mathrm{odo}}\ \mathbf{u}_b
$$

### 4.5 업데이트
$$
\mathbf{R}_{\mathrm{odo}}=\sigma_v^2\,\mathbf{I}_3,\qquad
\mathbf{K}=\mathbf{P}\mathbf{J}^\top(\mathbf{J}\mathbf{P}\mathbf{J}^\top+\mathbf{R}_{\mathrm{odo}})^{-1}
$$
$$
\delta\mathbf{x}=\mathbf{K}\,\mathbf{r}^{(v)},\qquad
\mathbf{P}\leftarrow(\mathbf{I}-\mathbf{K}\mathbf{J})\mathbf{P}
$$

### 4.6 ZUPT(자동)
$$
v_{\mathrm{odo}}\approx 0\ \Rightarrow\
\mathbf{r}^{(v)}_{\mathrm{ZUPT}}=-\mathbf{R}^\top\mathbf{v}
$$

## 5. 비동기 융합

- IMU: 고주기 전파
- 오도미터: 수신 시 EKF 갱신 (필요 시 \( r_{\mathrm{mag}} \) 병행)
- LiDAR: 스캔 시 IESKF 반복 갱신

## 6. 로버스트화

- 마할라노비스 게이팅
$$
\rho=\mathbf{r}^\top\mathbf{R}^{-1}\mathbf{r},\ \ \rho>\tau\Rightarrow\ \text{reject/down-weight}
$$
- IRLS(라이다), \( \|\delta\mathbf{x}\| \) trust-region
- 라이다 품질 저하시 오도미터 가중 상향

## 7. 관측성/튜닝

- 방향형 잔차는 \( \mathbf{R}^\top\mathbf{v} \)를 직접 구속 → roll/pitch 및 a-bias 유발 속도발산 억제\n- 크기-전용 잔차는 경사/자세 오인에 안전하나 자세 제약 약화\n- \( s_v \)는 느린 랜덤워크, 초기 1 근방, \( \sigma_v \)는 센서 스펙 기반