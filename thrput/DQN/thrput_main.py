#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
from ns3gym import ns3env

import numpy as np
from dqn_agent import DQNAgent
from utils import plot_learning_curve, make_env

if __name__ == '__main__':
    
    
    parser = argparse.ArgumentParser(description='Start simulation script on/off')
    parser.add_argument('--start',
                        type=int,
                        default=1,
                        help='Start ns-3 simulation script 0/1, Default: 1')
    # parser.add_argument('--iterations',
    #                     type=int,
    #                     default=3,
    #                     help='Number of iterations, Default: 1')
    
    args = parser.parse_args()
    startSim = bool(args.start)
    # iterationNum = int(args.iterations)
    
    port = 5555
    simTime = 5 # seconds
    stepTime = 0.5  # seconds
    seed = 0
    simArgs = {"--simTime": simTime,
               "--stepTime": stepTime,
               "--testArg": 123}
    debug = False
    
    env = ns3env.Ns3Env(port=port, stepTime=stepTime, startSim=startSim, simSeed=seed, simArgs=simArgs, debug=debug)

    obs = env.reset()
    #print("---obsinit: ", type(obs), obs)  # obs is tuple type, has no shape
    
    
    obs_np = np.reshape(obs, (4,))
    # print('obs_np: ',obs_np)   # obs_np:  [[0.42619916 0.42588068 0.02377036 0.14919516]]
    # print('more:', type(obs_np), obs_np.shape)
    
    
    ob_space = env.observation_space
    ac_space = env.action_space
    # print("Observation space: ", ob_space,  ob_space.dtype)     # Observation space:  Dict(box:Box(4,)) None
    # print("obs_space type: ", type(ob_space))                   #obs_space type:  <class 'gym.spaces.dict.Dict'>  #ob_space shape:  None
                      
    # print("Action space: ", ac_space, ac_space.dtype)      # Action space:  Dict(discrete:Discrete(9)) None
    # print("ac_space type: ", type(ac_space))                #ac_space type:  <class 'gym.spaces.dict.Dict'>  #ac_space shape:  None

  
    # print("obs: ",obs, type(obs))   #tuple  type [x,x,x,x]
    # print("obs[0]: ",  type(obs[0]), obs[0])        #obs[0]:  <class 'google.protobuf.pyext._message.RepeatedScalarContainer'> [0.18315342131954165, 0.33706036907000253, 1.0543681881024691, 0.8447054032585108]
    
    # # print("ac_space.discrete: ", ac_space.discrete)   #failed!
    # action = env.action_space.sample()      #test
    # print("action: ", action)   #action:  OrderedDict([('discrete', 4)])  #action has no dtype. it's a 'collections.OrderedDict' object 
    # print("action_: ", type(action))  #action_:  <class 'collections.OrderedDict'>
    
    
    best_score = -np.inf
    load_checkpoint = False
    n_games = 200       #200
    
    number_of_actions = 9  #0-8
    
    # agent = DQNAgent(gamma=0.99, epsilon=0.8, lr=0.0001,
    #                   input_dims=(env.observation_space.shape),
    #                   n_actions=env.action_space.n, mem_size=20000, eps_min=0.05,
    #                   batch_size=32, replace=1000, eps_dec=1e-5,
    #                   chkpt_dir='models/', algo='DQNAgent',
    #                   env_name='thrput4')
    
    agent = DQNAgent(gamma=0.99, epsilon=0.8, lr=0.0001,
                  input_dims=(obs_np.shape),
                  n_actions=number_of_actions, mem_size=20000, eps_min=0.05,
                  batch_size=32, replace=1000, eps_dec=1e-5,
                  chkpt_dir='models/', algo='DQNAgent',
                  env_name='thrput4')
    
    # print('observation: ', env.observation_space.shape,'n_actions: ', env.action_space.n)
    # print('observation: ', env.observation_space.shape)
    # print('actions: ', env.action_space)
    
    if load_checkpoint:
        agent.load_models()

    # fname = agent.algo + '_' + agent.env_name + '_lr' + str(agent.lr) +'_' \
    #         + str(n_games) + 'games'
    # figure_file = 'plots/' + fname + '.png'

    n_steps = 0
    scores, eps_history, steps_array = [], [], []    
    
#------------------------------------------------------------------------------   

        
#-----------------------------------------------------------------------------        
    for i in range(n_games):
        done = False
        observation = env.reset()
        observation_np = np.reshape(observation, (4,))
        # print("observation: ", type(observation),  observation)
        # print("observation_np: ", type(observation_np), observation_np.shape, observation_np)
        score = 0
        while not done:
            action_agent = agent.choose_action(observation_np)
            # print('take action:' , type(action_agent), action_agent)
            print('$take action: ' , action_agent)
            action = {"discrete":action_agent}
            observation_, reward, done, info = env.step(action)
            observation_np_ = np.reshape(observation_, (4,))
            score += reward

            if not load_checkpoint:
                agent.store_transition(observation_np, action_agent,
                                     reward, observation_np_, int(done))
                agent.learn()
            observation = observation_
            observation_np = observation_np_
            n_steps += 1
        scores.append(score)
        steps_array.append(n_steps)

        avg_score = np.mean(scores[-30:])
        print('episode: ', i,'score: ', score,
             ' average score %.1f' % avg_score, 'best score %.2f' % best_score,
            'epsilon %.2f' % agent.epsilon, 'steps', n_steps)

        if avg_score > best_score:
            if not load_checkpoint:
                agent.save_models()
            best_score = avg_score

        eps_history.append(agent.epsilon)
        if load_checkpoint and n_steps >= 18000:
            break

    # x = [i+1 for i in range(len(scores))]
    # plot_learning_curve(steps_array, scores, eps_history, figure_file)
    env.close()
    print("Done")        
        
        