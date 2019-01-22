# Target Generator
> Sistema de geração de targets para exploração autônoma de robôs móveis.

De um a dois parágrafos sobre o que é seu projeto e o que ele faz.

![](../header.png)

## Exemplo de uso

Inicie o ros:

```sh
roscore
```

Configure o parâmetro use_sim_time como true ().

```sh
rosparam set use_sim_time true
```

Inicie nodes necessários dos pacotes gmapping, ros navigation e youbot_navigation:

```sh
roslaunch target_generator target_generator.launch
```
Inicie o node gerador de targets:

```sh
rosrun target_generator target_generator
```

Abra o V-REP e inicie a simulação.

## Configuração para Desenvolvimento

Clone o projeto do github na pasta src de seu ros workspace:

```sh
cd (your_ros_ws)/src
git clone https://github.com/yourname/github-link
```
Compile (Recomendo usar [Catkin Command Line Tools](http://mcs.une.edu.au/doc/python-catkin_tools-doc/html/)):

```sh
cd ..
catkin build
```

## Meta

Distribuído sob a licença XYZ. Veja `LICENSE` para mais informações.

[https://github.com/yourname/github-link](https://github.com/othonalberto/)

###### Contribuidores:

Raphael Gomes – raphaelgoms@gmail.com

## Contributing

1. Faça o _fork_ do projeto (<https://github.com/yourname/yourproject/fork>)
2. Crie uma _branch_ para sua modificação (`git checkout -b feature/fooBar`)
3. Faça o _commit_ (`git commit -am 'Add some fooBar'`)
4. _Push_ (`git push origin feature/fooBar`)
5. Crie um novo _Pull Request_
