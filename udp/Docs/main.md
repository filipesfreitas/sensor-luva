
# Luva eletrônica

### Sistema #######

Esta divisão do trabalho vem da forma como foi concebido o projeto, dividindo o problema em tarefas. Cada tarefa representa um aspecto do projeto: comunicação entre microcontrolador e plataforma externa, controle da multiplexação dos canais e leitura dos registradores de cada canal.A sincronização destas tarefas é feita através de eventos em grupo ou EventGroup, no FREE RTOS bits de eventos são utilizados para indicar a ocorrência de um evento,é possível bloquear uma tarefa pela espera de um evento ou conjunto de eventos por um tempo pré determinado. Foram definidos 5 eventos que determinam as tarefas executadas no momento pelo dispositivo:

- Item1
 + 
- Item 2
