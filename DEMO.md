# Robot Navigator 데모 가이드

## 🎬 데모 시나리오

### 시나리오 1: 기본 시뮬레이터 테스트

**목적**: 시뮬레이터가 정상 작동하는지 확인

**실행**:
```bash
ros2 run robot_navigator simulator
```

**확인 사항**:
1. ✅ Pygame 창이 열림
2. ✅ 파란색 로봇이 화면 중앙에 표시
3. ✅ 화살표 키로 로봇 이동 가능
4. ✅ 정보 패널에 위치/속도 실시간 업데이트

**성공 기준**: 모든 항목 체크

---

### 시나리오 2: 레이저 센서 시각화

**목적**: 360도 레이저 센서 확인

**실행**:
```bash
ros2 run robot_navigator simulator
# 시뮬레이터 창에서 'L' 키 누르기
```

**확인 사항**:
1. ✅ 빨간색 레이저 빔 360개 표시
2. ✅ 장애물 감지 지점에 초록색 점
3. ✅ 벽과 장애물이 올바르게 감지됨

**추가 테스트**:
```bash
# 새 터미널에서
ros2 topic echo /scan
# LaserScan 데이터 확인
```

---

### 시나리오 3: 자동 내비게이션 (메인!)

**목적**: 장애물 회피하며 목표 도달

**실행**:
```bash
ros2 launch robot_navigator obstacle_avoidance.launch.py
```

**확인 사항**:
1. ✅ 시뮬레이터 창이 열림
2. ✅ 장애물들이 배치됨 (원형 5개 + 사각형 3개 + 벽)
3. ✅ 로봇이 자동으로 움직임
4. ✅ 장애물 근처에서 회피 동작
5. ✅ 목표 지점(6m, 4m)에 도달
6. ✅ 터미널에 "Goal Reached!" 메시지

**관찰 포인트**:
- 로봇이 장애물에 부딪히지 않음
- 부드러운 경로 생성
- 레이저가 장애물 감지하면 방향 전환

---

### 시나리오 4: ROS2 토픽 모니터링

**목적**: ROS2 통신 확인

**실행**:
```bash
# 터미널 1: 시뮬레이터 실행
ros2 launch robot_navigator obstacle_avoidance.launch.py

# 터미널 2: 토픽 확인
ros2 topic list

# 터미널 3: 위치 추적
ros2 topic echo /robot/pose

# 터미널 4: 레이저 데이터
ros2 topic echo /scan
```

**확인 사항**:
- `/cmd_vel` ✅
- `/robot/pose` ✅
- `/robot/odom` ✅
- `/scan` ✅

---

## 📊 성능 측정

### 테스트 방법

1. **목표 도달 성공률** (10회 반복)
```bash
# 10번 실행해서 몇 번 성공하는지 체크
for i in {1..10}; do
  echo "Test $i"
  ros2 launch robot_navigator obstacle_avoidance.launch.py
  # 목표 도달하면 Ctrl+C
done
```

2. **평균 도달 시간**
- 시작부터 "Goal Reached!" 메시지까지 시간 측정
- 목표: 30초 이내

3. **충돌 횟수**
- 장애물과 충돌 관찰
- 목표: 0회

---

## 🎯 데모 체크리스트

발표/시연 전 확인:

- [ ] ROS2 환경 source 완료
- [ ] 모든 패키지 최신 빌드
- [ ] 터미널 2-3개 준비
- [ ] 시뮬레이터 창 크기 적절히 조정
- [ ] 네트워크/퍼포먼스 안정적

시연 중:
- [ ] 시나리오 1: 기본 제어 (1분)
- [ ] 시나리오 2: 센서 시각화 (1분)
- [ ] 시나리오 3: 자동 내비게이션 (2-3분)
- [ ] 질의응답 준비

---

## 🐛 트러블슈팅

### 문제: 로봇이 목표에 도달하지 못함

**원인**: 지역 최소값(local minima)에 갇힘

**해결**:
- 장애물 배치 재생성 (프로그램 재시작)
- `safe_distance` 증가

### 문제: 레이저가 보이지 않음

**해결**: 
- 'L' 키 눌러서 토글
- `self.show_laser = True` 확인

### 문제: 로봇이 너무 느림

**해결**:
```python
# obstacle_avoidance.py
self.max_linear_speed = 0.8  # 0.5 → 0.8
```

---

**데모 준비 완료!** 🎉
