## Compilazione
Per compilare, eseguire tutta la prassi già spiegata dal prof tenendo a mente che **solo la prima volta** è necessario compilare con
```bash
colcon build
```
e poi dalle volte successive è sufficiente
```bash
colcon build --packages-select cpp_pubsub
```
Questo perchè ho definito separatamente dal sorgente del nodo un messaggio custom che deve compilato esplicitamente la prima volta; poi dopo si può compilare solo il codice del nodo cpp_pubsub (nel caso vengano fatte delle modifiche al codice di quest'ultimo).
Si può lanciare ogni volta il `colcon build` senza parametri ma dovendo ricompilare tutto si perde tempo inutilmente.
## Esecuzione
Per eseguire è sufficiente lanciare in un terminale
```bash
source install/setup.bash
ros2 run cpp_pubsub talker
```
e in un altro
```bash
source install/setup.bash
ros2 run cpp_pubsub listener
```
Per il grafico, una volta lanciati i nodi di cui sopra, aprire un terzo terminale e lanciare
```bash
source install/setup.bash
ros2 run rqt_plot rqt_plot
```
Una volta aperto, digitare `/time/time` come riportato nell'immagine e premere il tasto invio assicurandoti poi di aver selezionato *autoscroll* altrimenti non vedete il grafico (sì, nell'immagine non è selezionato ma non mi andava di rifare lo screen).
![Screenshot 2023-05-06 103646](https://user-images.githubusercontent.com/4050967/236613412-d6ec97e8-a670-480c-b0c4-2db3c2668aaa.png)

## Lanciare gym_ros
Eseguire:
```bash
cd sim/f1tenth_gym_ros
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

## Lanciare ftg_node
[Scaricare](https://github.com/HiPeRT/F1tenth-RTES-FTG) ed eseguire:
```bash
cd sim/F1tenth-Rtes-FTG
source install/setup.bash
ros2 launch ftg ftg_node.launch.xml
```
