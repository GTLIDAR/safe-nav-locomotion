import simplejson as json
from gridworld import Gridworld

# must manually add }} to the end of the output file

class BeliefIOParser():
    def __init__(self, filename = "temp.json", read = False):
        self.filename = filename
        self.file_handle = None

        # Are we reading or writing a file?
        self.read = read
        self.step_counter = 0;
        if(self.read): # reading
            self.file_handle = open(filename, 'r')
            self.loaded_data = json.load(self.file_handle)
        else: # writing
            self.file_handle = open(filename, 'w')
            self.file_handle.write('{\"saved_states\": {')

    def saveState(self, gwg, automaton, automaton_state, gridstate, moveobstacles, gwg_c):
        env_state = [0 for i in range(len(moveobstacles))]
        try:
            for n in range(0,len(moveobstacles)):
                env_state[n] = automaton[automaton_state]['State']['st{}'.format(n)]
        except:
            env_state = automaton[automaton_state]['State']['st']
        try:
            agent_state = automaton[automaton_state]['State']['s_c']
        except:
            agent_state = automaton[automaton_state]['State']['s']

        agent_location = gwg.coords(agent_state) 
        # env_location = gwg.coords(env_state)
        # try:
        #     obs_location = [0 for i in range(len(moveobstacles))]
        #     for n in range(0,len(moveobstacles)):
        #         obs_location[n] = gwg.coords(gridstate[n])
        # except:
        #     obs_location = gwg.coords(gridstate)
        
        obs_location = []
        for n in range(0,len(moveobstacles)):
            try:
                a, b = gwg_c.coords(gridstate[n])
                obs_location.append(a)
                obs_location.append(b)
            except:
                obs_location = gwg_c.coords(gridstate)

        output = json.dumps({'agent_state': agent_state, 'agent_location': agent_location, \
            'env_state': env_state, 'obstacle_location': obs_location, \
            'automaton_state': automaton_state, 'action_info': automaton[automaton_state]})

        self.file_handle.write("\"" + str(self.step_counter) + "\":" + output + ',\n')
        self.step_counter += 1

    def advanceStep(self):
        self.step_counter += 1

    def setStep(self, step):
        self.step_counter = step

    def getStep(self):
        return self.step_counter

    def getTotalSteps(self):
        return len(self.loaded_data["saved_states"])

    def getProperty(self, name, step = -1):
        if(step < 0):
            step = self.step_counter
        try: # Terrible but it works
            return self.loaded_data["saved_states"][str(step)][name]
        except:
            return self.loaded_data["saved_states"][str(step)]["action_info"]["State"][name] 

    def getLoadedData(self):
        return self.loaded_data
