def centroid(readings):
    weights = [-2.0, -1.0, 0.0, 1.0, 2.0]
    
    sum_readings = sum(readings)
    weighted_sum = sum(r * w for r, w in zip(readings, weights))
    
    if sum_readings == 0.0: #boundary case
        return 11 #returning smth out of -2 to 2 so that we know at this sum was 0 and we can program accordingly.
        
    return weighted_sum / sum_readings


def motor_clamp(left_motor, right_motor, max_magnitude):# generally max_magnitude = 255
    left_clamped = max(-max_magnitude, min(left_motor, max_magnitude))
    right_clamped = max(-max_magnitude, min(right_motor, max_magnitude))
    
    return left_clamped, right_clamped


normal = [10, 50, 100, 20, 0]
print(f"Centroid: {centroid(normal):.3f}") # Output: -0.278
    
zero = [0, 0, 0, 0, 0]
print(f"Centroid: {centroid(zero)}") # Output: 11 (special case)

print(f"Centroid: {centroid([100, 200, 300 ,100,100])}") 

print(motor_clamp(100 , -300 , 255))
print(motor_clamp(100 , -200 , 255))

"""
Logic Explanation:
1. Line-Position Calculation: centoid is calculated by multiplying sensor reading with weight(-2, -1 , 0 , 1 , 2) , this gives us more precise value that when if we would have used only is/else , more precise value means more control over how much to turn and all . Also if it is 0 , then we know our robot is perfectly on line , and when 11 , we know that sum of all reading came 0 and we can program ahead accordingly.
2. Motor-Command Limiting: motor speed can't be more than 255 due to 8 bit timer so we have clamped so it reaches max absolute value of 255. 
"""