import rospy
from controller.enemy_controller import EnemyController
from controller.dorlion_controller import DorlionController
from findDangerPlane.fuzzy_inference_system import FIS
import time
import path_planning.aStar as aStar
import numpy as np
import path_planning.dijkstra as dijkstra

def main():


    rospy.init_node('fuzzyInferenceSystem_node'.format(1), anonymous=True)

    uav = DorlionController()
    enemys = []
    enemy_names = ["uav1", "uav2","uav3","uav4"]
    for eName in enemy_names:
        enemy = EnemyController(eName)
        enemy.takeoff()
        enemy.setDorlion(uav.getPosition())
        enemys.append(enemy)
        
    start_time = time.time()
    print("Düşman uçakları seyir halinde")

    while (time.time() - start_time) < 10:
        for enemy in enemys:
            enemy.set_randomly_speed()
    print("En Tehlikeli Uçak Bulunuyor")    
    print("*************************")
    fis_inference = []
    for i,enemy in enumerate(enemys):
        enemy.stop_uav()
        features = enemy.get_features()
        enemy_fis = FIS(features[0],features[1],features[2],features[3])
        print(f'{enemy_names[i]:} \nDistance: {features[0]}, Velocity: {features[1]},Position: {features[2]}',end="")
        inf_ = enemy_fis.inference()
        fis_inference.append(inf_)
        print("Danger Level:",inf_)
    print("*************************")
    print("Most Dangereous Plane : ",enemy_names[fis_inference.index(max(fis_inference))]," Danger Level : ",max(fis_inference))

    danger_enemy = enemys[fis_inference.index(max(fis_inference))] 
    start_ = (round(uav.getPosition().x),round(uav.getPosition().y),round(uav.getPosition().z))
    end_ = (round(danger_enemy.getPosition().x),round(danger_enemy.getPosition().y),round(danger_enemy.getPosition().z))
    obstacles = []
    for i in range(len(enemys)):
        if i != max(fis_inference):        
            obstacles.append((round(enemys[i].getPosition().x),round(enemys[i].getPosition().y),round(enemys[i].getPosition().z))) 
    
    obstacles = np.array(obstacles)
    print("***********************")
    print("Start Point: ",start_)
    print("End Point:",end_)
    print("****************")
    
    path_points = aStar.astar(start_,end_,obstacles,obstacle_radius=1)
    print("Path Points:",path_points)
    uav.setDangerUav(danger_enemy)
    uav.enemys = enemys
    uav.move_path(path_points)
    for i,enemy in enumerate(enemys):
        enemy.stop_uav()
    while True:
        uav.detection()
        uav.tracking()
        danger_enemy.set_randomly_speed(over = 11)
   
        # sys.exit(app.exec_())

if __name__ == "__main__":
    main()

