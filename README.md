# RMI

## * ParticleFilterSimple
### Dependências
* [Python 2.7](https://www.python.org/download/releases/2.7/)
* [Matplotlib](https://matplotlib.org/)
* Criar uma pasta chamada "output" no path onde o código se encontra
```
-- Terminal
cd /path_to_the_code
mkdir output
```

### Variáveis que podem ser alteradas
```
vim /path_to/particlefiltersimple.py

world_size       # tamanho do mapa (px)
num_of_particles # número de partículas no ambiente
num_obstacles    # número de obstaculos (Fake)
steps            # número de movimentos do robô
```
### Executando
Para executar, basta adicionar ao terminal o comando
```
cd /path_to_the_code
python particlefiltersimple.py
```
Você podera acompanhar a atualização do filtro pela criação das imagens na pasta output. Veja o demo abaixo para um melhor entendimento

#### Demo
* [Gif](https://imgflip.com/gif/211geq)


## * ParticleFilterROS
### Dependências
* [Python 2.7](https://www.python.org/download/releases/2.7/)
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [Gazebo 6](http://gazebosim.org/download)
* [Rviz](http://wiki.ros.org/rviz)

### Variáveis que podem ser alteradas
```
vim /path_to/pf_rmi_17.py

n_particles          # número de partículas
linear_mov           # movimentação linear (metros) para atualização do filtro
angular_mov          # movimentação angular para atualização do filtro
laser_max_distance   # leitura máxima do laser (metros)
rad                  # circunferência do filtro de partículas (metros)

--- Ruído
particle.x += d * (math.cos(particle.theta) + normal(0,0.01))
particle.y += d * (math.sin(particle.theta) + normal(0,0.01))
```
Pode-se alterar o nível de ruído das partículas alterando os valores da expressão acima. Basta alterar os valores dentro da função <i><b>normal(0,0.01)</b></i> para por ex. <i><b>normal(0,0.1)</b></i>. Quanto maior o valor de saída, maior sera o nível de ruído da partícula

### Executando
<i>Especificamente neste exemplo usaremos o Turtlebot porém o código pode ser rodado com qualquer robô que tenha odômetria e um laser instalado</i>

1. Abra o terminal e inicie o Turtlebot em Stage Mode
```
roslaunch turtlebot_stage turtlebot_stage.lauch
```
2. Dentro do Rviz adicione um novo PoseArray e preencha o tópico com
```
/particlecloud_rmi
```
3. Abra o terminal e rode o arquivo pf_rmi_17.py
```
cd /path_to_your_folder/scripts/
python pf_rmi_17.py

CTRL+C para interromper a execução
```

4. Insira goals no rviz (2D Nav Goal) para movimentar o robô, e veja o filtro de partículas se atualizando conforme o robô se movimenta  <br/><t/>Dica: para uma melhor visualização deve-se remover o PoseArray AMCL carregado por default em Stage
</br>
<b><i>Obs.</i></b> O projeto está arquiteturado para ser rodado também diretamente no ROS via catkin. Apenas siga os passos do ROS catkin e cmake, conforme documentação [ROS Catkin](http://wiki.ros.org/catkin)

#### Demos
<i>Aconselhável alterar a qualidade para HD 720p</i>
* [Filtro de Partículas com amostragem de 60 partículas](https://youtu.be/CEA7PzWgeRg)
* [Filtro de Partículas com amostragem de 120 partículas](https://youtu.be/sSDb32Uu2Pc)
* [Filtro de Partículas AMCL - Para Comparação](https://youtu.be/Emuxr_PubYA) - Padrão ROS (Sem mudanças)

