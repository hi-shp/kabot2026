# rf2o_laser_odometry

연속된 2D 라이다 스캔으로 평면 이동과 회전을 추정하는 패키지입니다. 바퀴 오도메트리의 정확도가 낮은 이동 로봇 등에 사용할 수 있습니다.

각 스캔 점의 거리 변화와 센서 속도 사이 제약식을 세우고, 이상값에 강한 비용함수를 최소화해 움직임을 구합니다. 스캔의 기울기 정보를 이용해 전체 점을 정렬하는 방식입니다.

원문에서 제시한 연산 시간은 CPU 단일 코어 기준 약 0.9ms입니다. 실제 처리 시간과 정확도는 장비와 주변 환경에 따라 달라집니다.

알고리즘은 ICRA 2016 논문 [Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach](http://mapir.uma.es/papersrepo/2016/2016_Jaimez_ICRA_RF2O.pdf)에 설명되어 있습니다.
