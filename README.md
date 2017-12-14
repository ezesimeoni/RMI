# RMI

## ParticleFilterSimple
### Dependências
* [Python 2.7](https://www.python.org/download/releases/2.7/)
* [Matplotlib](https://matplotlib.org/)
* Criar uma pasta chamada "output" no path onde o código se encontra

### Variaveis que podem ser alteradas
```
world_size       # tamanho do mapa
num_of_particles # numero de particulas no ambiente
num_obstacles    # numero de obstaculos (Fake)
steps            # numero de movimentos do robo
```
### Executando
Para executar basta adicionar ao terminal o comando
```
python particlefiltersimple.py
```
Você podera acompanhar a execução pela criação das imagens na pasta output
- O output terá imagens como a do <b>[Gif](https://imgflip.com/gif/211geq) aqui mostrado</b>


## ParticleFilterROS
### Dependências
* [Python 2.7](https://www.python.org/download/releases/2.7/)
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [Gazebo 6](http://gazebosim.org/download)
* [Rviz](http://wiki.ros.org/rviz)

### Executando
<i>Especificamente neste exemplo usaremos o Turtlebot porém o código pode ser rodado com qualquer robo que tenha odômetria e um laser instalado</i>

1. Inicie o Turtlebot em Stage
```
roslaunch turtlebot_stage turtlebot_stage.lauch
```
2. Dentro do Rviz adicione um novo PoseArray e preencha o tópico com
```
/pointcloud_rmi
```
3. Rode o código pf_rmi_17.py
```
cd /para/pasta/onde/esta/codigo
python pf_rmi_17.py
```
4. Insira goals para o robo e veja o filtro de partículas se movendo <br/><t/>Obs. Para uma melhor visualização deve-se remover o PoseArray AMCL carregado por default em Stage mode
