import numpy as np
import skfuzzy as fuzz
import skfuzzy.membership as mf

class FIS():
    
    def __init__(self,name,distance,velocity,position):
        self.velocity = velocity
        if self.velocity <= 0.0:
            self.velocity = 0.001
        if self.velocity >= 0.9:
            self.velocity = 0.899999
        self.distance = self.normalized_distance(distance)
        if self.distance >= 0.9:
            self.distance = 0.899999
        self.position = position
        self.name = name
        
        self.distance_env = np.arange(0, 1.0, 0.1)
        self.position_env = np.arange(0, 1.0, 0.1)
        self.velocity_env = np.arange(0, 1.1, 0.1)
        self.y_risk_env = np.arange(0, 1, 0.1)
        
        self.velocity = velocity

        self.distance_very_close = fuzz.trapmf(self.distance_env, [0, 0, 0.025, 0.1])
        self.distance_close = fuzz.trimf(self.distance_env, [0.05, 0.1, 0.2])
        self.distance_normal = fuzz.trimf(self.distance_env, [0.1, 0.2, 0.3])
        self.distance_far = fuzz.trimf(self.distance_env, [0.2, 0.3, 0.4])
        self.distance_very_far = fuzz.trimf(self.distance_env, [0.3, 0.4, 1.0])
        
        # self.distance_very_close = fuzz.trapmf(self.distance_env, [0, 0,0.5, 2])
        # self.distance_close = fuzz.trimf(self.distance_env, [1, 2, 4])
        # self.distance_normal = fuzz.trimf(self.distance_env, [2, 4, 6])
        # self.distance_far = fuzz.trimf(self.distance_env, [4, 6, 8])
        # self.distance_very_far = fuzz.trimf(self.distance_env, [6, 8, 20])

        self.position_not_danger = fuzz.trimf(self.position_env, [0, 0.2, 0.4])
        self.position_normal = fuzz.trimf(self.position_env, [0.2, 0.4, 0.6])
        self.posiiton_danger = fuzz.trimf(self.position_env, [0.4, 0.6, 0.8])
        self.posiiton_very_danger = fuzz.trimf(self.position_env, [0.6, 0.8, 1.0])

        self.velocity_low = fuzz.trimf(self.velocity_env, [0.0, 0.3, 0.3])
        self.velocity_normal = fuzz.trimf(self.velocity_env, [0.3, 0.6, 0.6])
        self.velocity_high = fuzz.trimf(self.velocity_env, [0.6, 1.0, 1.0])

        self.risk_not = mf.trapmf(self.y_risk_env, [0 ,0 ,0.1 ,0.2])
        self.risk_low = mf.trapmf(self.y_risk_env, [0.1 ,0.2 ,0.3 ,0.4])
        self.risk_normal = mf.trapmf(self.y_risk_env, [0.2 ,0.3 ,0.4 ,0.5])
        self.risk_danger = mf.trapmf(self.y_risk_env, [0.3 ,0.4 ,0.5 ,0.6])
        self.risk_high_danger = mf.trapmf(self.y_risk_env, [0.4, 0.5, 0.6, 0.7])
        self.risk_very_high_danger = mf.trapmf(self.y_risk_env, [0.5,0.6,0.8,1.0])

    def inference(self):

        self.distance_membership_very_close = fuzz.interp_membership(self.distance_env, self.distance_very_close, self.distance)
        self.distance_membership_close = fuzz.interp_membership(self.distance_env, self.distance_close, self.distance)
        self.distance_membership_normal = fuzz.interp_membership(self.distance_env, self.distance_normal, self.distance)
        self.distance_membership_far = fuzz.interp_membership(self.distance_env, self.distance_far, self.distance)
        self.distance_membership_very_far = fuzz.interp_membership(self.distance_env, self.distance_very_far, self.distance)

        self.position_membership_not_danger = fuzz.interp_membership(self.position_env, self.position_not_danger, self.position)
        self.position_membership_normal = fuzz.interp_membership(self.position_env, self.position_normal, self.position)
        self.position_membership_danger = fuzz.interp_membership(self.position_env, self.posiiton_danger, self.position)
        self.position_membership_very_danger = fuzz.interp_membership(self.position_env, self.posiiton_very_danger, self.position)
        
        self.velocity_membership_low = fuzz.interp_membership(self.velocity_low, self.velocity_low, self.velocity)
        self.velocity_membership_normal = fuzz.interp_membership(self.velocity_normal, self.velocity_normal, self.velocity)
        self.velocity_membership_high = fuzz.interp_membership(self.velocity_high, self.velocity_high, self.velocity)
                
        rule1 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_far, self.position_membership_not_danger),self.velocity_membership_high), self.risk_not)

        rule2 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_far, self.position_membership_normal),self.velocity_membership_high), self.risk_not)
        
        rule3 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_far, self.position_membership_danger),self.velocity_membership_high), self.risk_low)
        
        rule4 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_far, self.position_membership_very_danger),self.velocity_membership_normal), self.risk_normal)
        
        rule5 = np.fmin(np.fmin(self.distance_membership_very_far, np.fmin(self.position_membership_normal, self.velocity_membership_low)), self.risk_low)

        rule6 = np.fmin(np.fmin(self.distance_membership_very_far, np.fmin(self.position_membership_normal, self.velocity_membership_normal)), self.risk_low)

        rule7 = np.fmin(np.fmin(self.distance_membership_very_far, np.fmin(self.position_membership_normal, self.velocity_membership_normal)), self.risk_normal)

        rule8 = np.fmin(np.fmin(self.distance_membership_very_far, np.fmin(self.position_membership_very_danger, self.velocity_membership_normal)), self.risk_normal)

        rule9 = np.fmin(np.fmin(self.distance_membership_far, np.fmin(self.position_membership_not_danger, self.velocity_membership_low)), self.risk_low)
        
        rule10 = np.fmin(np.fmin(self.distance_membership_far, np.fmin(self.position_membership_normal, self.velocity_membership_low)), self.risk_low)

        rule11 = np.fmin(np.fmin(self.distance_membership_far, np.fmin(self.position_membership_normal, self.velocity_membership_normal)), self.risk_low)
        
        rule12 = np.fmin(np.fmin(self.distance_membership_far, np.fmin(self.position_membership_normal, self.velocity_membership_normal)), self.risk_normal)

        rule13 = np.fmin(np.fmin(self.distance_membership_far, np.fmin(self.position_membership_danger, self.velocity_membership_low)), self.risk_normal)
        
        rule14 = np.fmin(np.fmin(self.distance_membership_far, np.fmin(self.position_membership_danger, self.velocity_membership_normal)), self.risk_danger)
        
        rule15 = np.fmin(np.fmin(self.distance_membership_far, np.fmin(self.position_membership_danger, self.velocity_membership_normal)), self.risk_high_danger)
        
        rule16 = np.fmin(np.fmin(self.distance_membership_far, np.fmin(self.position_membership_very_danger, self.velocity_membership_low)), self.risk_danger)
        
        rule17 = np.fmin(np.fmin(self.distance_membership_far, np.fmin(self.position_membership_very_danger, self.velocity_membership_normal)), self.risk_high_danger)
        
        rule18 = np.fmin(np.fmin(self.distance_membership_far, np.fmin(self.position_membership_very_danger, self.velocity_membership_normal)), self.risk_very_high_danger)

        rule19 = np.fmin(np.fmin(self.distance_membership_normal, np.fmin(self.position_membership_not_danger, self.velocity_membership_low)), self.risk_low)

        rule20 = np.fmin(np.fmin(self.distance_membership_normal, np.fmin(self.position_membership_normal, self.velocity_membership_low)), self.risk_normal)

        rule21 = np.fmin(np.fmin(self.distance_membership_normal, np.fmin(self.position_membership_normal, self.velocity_membership_normal)), self.risk_danger)

        rule22 = np.fmin(np.fmin(self.distance_membership_normal, np.fmin(self.position_membership_danger, self.velocity_membership_normal)), self.risk_danger)
        
        rule23 = np.fmin(np.fmin(self.distance_membership_normal, np.fmin(self.position_membership_danger, self.velocity_membership_normal)), self.risk_high_danger)
    
        rule24 = np.fmin(np.fmin(self.distance_membership_normal, np.fmin(self.position_membership_very_danger, self.velocity_membership_low)), self.risk_danger)
        
        rule25 = np.fmin(np.fmin(self.distance_membership_normal, np.fmin(self.position_membership_very_danger, self.velocity_membership_normal)), self.risk_high_danger)
        
        rule26 = np.fmin(np.fmin(self.distance_membership_normal, np.fmin(self.position_membership_very_danger, self.velocity_membership_normal)), self.risk_very_high_danger)
            
        rule27 = np.fmin(np.fmin(np.fmin(self.distance_membership_close, self.position_membership_not_danger),self.velocity_membership_high), self.risk_low)

        rule28 = np.fmin(np.fmin(self.distance_membership_close, np.fmin(self.position_membership_normal, self.velocity_membership_low)), self.risk_normal)
        
        rule29 = np.fmin(np.fmin(self.distance_membership_close, np.fmin(self.position_membership_normal, self.velocity_membership_normal)), self.risk_danger)
        
        rule30 = np.fmin(np.fmin(self.distance_membership_close, np.fmin(self.position_membership_normal, self.velocity_membership_normal)), self.risk_high_danger)
        
        rule31 = np.fmin(np.fmin(self.distance_membership_close, np.fmin(self.position_membership_danger, self.velocity_membership_low)), self.risk_danger)
        
        rule32 = np.fmin(np.fmin(self.distance_membership_close, np.fmin(self.position_membership_danger, self.velocity_membership_normal)), self.risk_high_danger)
        
        rule33 = np.fmin(np.fmin(self.distance_membership_close, np.fmin(self.position_membership_danger, self.velocity_membership_normal)), self.risk_very_high_danger)
        
        rule34 = np.fmin(np.fmin(np.fmin(self.distance_membership_close, self.position_membership_very_danger),self.velocity_membership_high), self.risk_very_high_danger)

        rule35 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_close, self.position_membership_not_danger),self.velocity_membership_high), self.risk_normal)

        rule36 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_close, self.position_membership_normal),self.velocity_membership_high), self.risk_high_danger)

        rule37 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_close,self.position_membership_danger),self.velocity_membership_high),self.risk_very_high_danger)
        
        rule38 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_close,self.position_membership_very_danger),self.velocity_membership_high),self.risk_very_high_danger)

        rule39 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_far, self.velocity_membership_low),self.velocity_membership_high), self.risk_not)

        rule40 = np.fmin(np.fmin(self.distance_membership_far, np.fmin(self.position_membership_normal, self.velocity_membership_normal)), self.risk_normal)

        rule41 = np.fmin(np.fmin(self.distance_membership_normal, np.fmin(self.position_membership_normal, self.velocity_membership_normal)), self.risk_normal)

        rule42 = np.fmin(np.fmin(self.distance_membership_normal, np.fmin(self.position_membership_danger, self.velocity_membership_normal)), self.risk_danger)

        rule43 = np.fmin(np.fmin(self.distance_membership_close, np.fmin(self.position_membership_normal, self.velocity_membership_normal)), self.risk_danger)

        rule44 = np.fmin(np.fmin(np.fmin(self.distance_membership_far, self.position_membership_not_danger),self.velocity_membership_high), self.risk_not)

        rule45 = np.fmin(np.fmin(np.fmin(self.distance_membership_normal, self.position_membership_not_danger),self.velocity_membership_high), self.risk_low)
        
        rule46 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_far, self.position_membership_not_danger),self.velocity_membership_low), self.risk_not)

        rule47 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_far, self.position_membership_not_danger),self.velocity_membership_normal), self.risk_not)

        rule48 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_far, self.position_membership_normal),self.velocity_membership_low), self.risk_not)
        
        rule49 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_far, self.position_membership_normal),self.velocity_membership_normal), self.risk_not)
        
        rule50 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_far, self.position_membership_danger),self.velocity_membership_normal), self.risk_low)
        
        rule51 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_far, self.position_membership_danger),self.velocity_membership_low), self.risk_low)

        rule52 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_far, self.position_membership_very_danger),self.velocity_membership_high), self.risk_normal)
        
        rule53 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_far, self.position_membership_very_danger),self.velocity_membership_low), self.risk_normal)

        rule54 = np.fmin(np.fmin(np.fmin(self.distance_membership_close, self.position_membership_not_danger),self.velocity_membership_low), self.risk_low)

        rule55 = np.fmin(np.fmin(np.fmin(self.distance_membership_close, self.position_membership_not_danger),self.velocity_membership_normal), self.risk_low)

        rule56 = np.fmin(np.fmin(np.fmin(self.distance_membership_close, self.position_membership_very_danger),self.velocity_membership_low), self.risk_very_high_danger)

        rule57 = np.fmin(np.fmin(np.fmin(self.distance_membership_close, self.position_membership_very_danger),self.velocity_membership_normal), self.risk_very_high_danger)

        rule58 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_close, self.position_membership_not_danger),self.velocity_membership_low), self.risk_normal)

        rule59 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_close, self.position_membership_not_danger),self.velocity_membership_normal), self.risk_normal)

        rule60 = np.fmin(np.fmin(np.fmin(self.distance_membership_far, self.position_membership_not_danger),self.velocity_membership_low), self.risk_not)

        rule61 = np.fmin(np.fmin(np.fmin(self.distance_membership_far, self.position_membership_not_danger),self.velocity_membership_normal), self.risk_not)

        rule62 = np.fmin(np.fmin(np.fmin(self.distance_membership_normal, self.position_membership_not_danger),self.velocity_membership_low), self.risk_low)

        rule63 = np.fmin(np.fmin(np.fmin(self.distance_membership_normal, self.position_membership_not_danger),self.velocity_membership_normal), self.risk_low)

        rule64 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_close, self.position_membership_normal),self.velocity_membership_low), self.risk_high_danger)

        rule65 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_close, self.position_membership_normal),self.velocity_membership_normal), self.risk_high_danger)

        rule66 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_close,self.position_membership_danger),self.velocity_membership_low),self.risk_very_high_danger)

        rule67 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_close,self.position_membership_danger),self.velocity_membership_normal),self.risk_very_high_danger)

        rule68 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_close,self.position_membership_very_danger),self.velocity_membership_low),self.risk_very_high_danger)

        rule69 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_close,self.position_membership_very_danger),self.velocity_membership_normal),self.risk_very_high_danger)

        rule70 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_far, self.velocity_membership_low),self.velocity_membership_low), self.risk_not)

        rule71 = np.fmin(np.fmin(np.fmin(self.distance_membership_very_far, self.velocity_membership_low),self.velocity_membership_normal), self.risk_not)

        out_not = np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(rule1,rule2),rule39),rule44),rule45),rule46),rule47),rule48),rule49),rule60),rule61),rule70),rule71)
        out_low = np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(rule3,rule5),rule6),rule9),rule10),rule11),rule19),rule27),rule45),rule50),rule51),rule54),rule55),rule62),rule63)
        out_normal = np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(rule4,rule7),rule8),rule12),rule13),rule20),rule28),rule35),rule40),rule41),rule52),rule53),rule58),rule59)
        out_danger = np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(rule14,rule16),rule21),rule22),rule24),rule29),rule31),rule42),rule43)
        out_high_danger = np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(rule15,rule17),rule23),rule25),rule30),rule32),rule36),rule64),rule65)
        out_very_high_danger = np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(rule18,rule26),rule33),rule34),rule37),rule38),rule56),rule57),rule66),rule67),rule68),rule69)
        
        self.out_risk = np.fmax(np.fmax(np.fmax(np.fmax(np.fmax(out_not,out_low),out_normal),out_danger),out_high_danger),out_very_high_danger)
        self.defuzzified = fuzz.defuzz(self.y_risk_env,self.out_risk,'centroid')
        result = fuzz.interp_membership(self.y_risk_env, self.out_risk,self.defuzzified)

        return result


    def normalized_distance(self,value):
        return ((value-0)/(20-0))*(1-0) + 0
