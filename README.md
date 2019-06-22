# ufmg_experiments





## Planejamento do experimento na UFMG


Obter os códigos desse repositório aqui (na nuc do robô):

`git clone https://github.com/adrianomcr/ufmg_experiments.git`

E dar um:

`catkin build`

Colocar o robô virado para o leste, na posição 000 (x, y, yaw) desejada.

Rodar todos os .launch que foram rodados no experimento da quarta-feira, exceto o do joystick. A gui não é essencial mas pode ser chamada também.

`roslaunch TUDO QUE FOI RODADO ANTES MENOS JOYSTICK`

Rodar o seguinte launch:

`roslaunch ufmg_experiments experimento_campo_futebol.launch`

Esse launch tem dois nós:
- pose_constructor - vai setar o "000" do robô e gerar a pose relativa com base nos dados do GPS e da IMU
- vec_field_ufmg.py - vai esperar uma trajetória e quando recebe-la vai gerar comandos /cmd_vel para o robô

Esperar um tempinho (um segundo pelo menos). Isso serve para o código setar o 000. Esse um tempinho significa: "O código a seguir não deve ser chamado no mesmo launch dos nós acima, e sim depois."

Rodar o códico que vai gerar uma trajetória exemplo.

`rosrun espeleo_planning example_trajectories_ufmg.py N_CURVE N_POINTS`

- N_CURVE = 1  =>  oito
- N_CURVE = 2  =>  elipse
- N_CURVE = 3  =>  "quadrado" sem quinas

Recomendo N_POINTS = 1000 ~ 2000.

Se tudo der certo o robô começa a andar e seguir a curva.

