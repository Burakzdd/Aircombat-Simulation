from UAVEnvironment import UAVEnvironment
from UAVAgent import UAVAgent
import rospy


def main():

    env = UAVEnvironment()
    plane = UAVAgent()
    
    total_reward = 0
    episodes = 100

    for e in range(episodes):
        env.clear()
        env.takeoff()
        
        print("*********************************************************")
        
        total_reward = 0
        
        state = env.get_state()
        env.set_target_position()
        

        for step in range(env.max_steps):
            action = plane.findAction(state)
            next_state, reward, done = env.step(action)

            plane.remember(state, action, reward, next_state,done)
            plane.replay()
            
            state = next_state
            total_reward += reward
            if abs(next_state[0]) > 20 :
                break
            
            if done:
                break
        # print("Episode: {}/{}, Total Reward: {}".format(e , episodes, total_reward))
        print(reward)

        
        
        
if __name__ == "__main__":
    rospy.init_node('RL_node', anonymous=True)
    main()