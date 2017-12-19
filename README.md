# RMI

## ParticleFilterSimple
### Dependências
* [Python 2.7](https://www.python.org/download/releases/2.7/)
* [Matplotlib](https://matplotlib.org/)
* Criar uma pasta chamada "output" no path onde o código se encontra

### Variaveis que podem ser alteradas
```
world_size       # tamanho do mapa
num_of_particles # numero de partículas no ambiente
num_obstacles    # numero de obstaculos (Fake)
steps            # numero de movimentos do robo
```
### Executando
Para executar basta adicionar ao terminal o comando
```
python particlefiltersimple.py
```
Você podera acompanhar a execução pela criação das imagens na pasta output

#### Demo
* [Gif](https://imgflip.com/gif/211geq)


## ParticleFilterROS
### Dependências
* [Python 2.7](https://www.python.org/download/releases/2.7/)
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [Gazebo 6](http://gazebosim.org/download)
* [Rviz](http://wiki.ros.org/rviz)

### Executando
<i>Especificamente neste exemplo usaremos o Turtlebot porém o código pode ser rodado com qualquer robô que tenha odômetria e um laser instalado</i>

1. Inicie o Turtlebot em Stage
```
roslaunch turtlebot_stage turtlebot_stage.lauch
```
2. Dentro do Rviz adicione um novo PoseArray e preencha o tópico com
```
/particlecloud_rmi
```
3. Rode o arquivo pf_rmi_17.py
```
cd /path_to_your_folder/scripts/
python pf_rmi_17.py
```
<i>Para interromper a execução CTRL+C</i></br></br>
4. Insira goals no rviz (2D Nav Goal) para movimentar o robô, e veja o filtro de partículas se atualizando conforme o robô se movimenta  <br/><t/>Dica: para uma melhor visualização deve-se remover o PoseArray AMCL carregado por default em Stage
</br>
* <i>Obs. O projeto está arquiteturado para ser rodado também diretamente no ROS via catkin. Apenas siga os passo do ROS catkin e cmake conforme site do [ROS](http://wiki.ros.org/catkin)</i>

#### Demos
* [Filtro de Partículas com amostragem de 60 partículas](https://youtu.be/CEA7PzWgeRg)
* [Filtro de Partículas com amostragem de 120 partículas](https://youtu.be/sSDb32Uu2Pc)
* [Filtro de Partículas AMCL - Para Comparação](https://youtu.be/Emuxr_PubYA) - Padrão ROS (Sem mudanças)

