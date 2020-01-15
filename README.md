# Projekt---Sterowanie-Robotem-Kroczacym

# Do uruchomienia niezbędne są:
- ROS Melodic, packages: HumaRobotics darwin_gazebo, darwin_description, darwin_control
- Gazebo
- MatLab, biblioteki: ROS Toolbox, Robotics System Toolbox 

# Uruchomienie
1. Włączenie symulacji w Gazebo:
```
roslaunch darwin_gazebo darwin_gazebo.launch
```
2. Rozpoczęcie symulacji przyciskiem Play
3. Wstawienie przeszkód w postaci obiektów "Sphere", na płaszczyźnie wytyczonej pomiędzy zieloną i czerwoną osią w Gazebo. Przeszkoda nie może znajdować się na punkcie celu dla robota (domyślnie: robotGoal = [5, 8])
4. Uruchomienie skryptu "sterowanie_robot" w MatLab
