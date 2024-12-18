import math

# Helper function to determine direction based on "o'clock"
def oclock_to_degrees(oclock):
    # The 12 o'clock position is straight ahead (0 degrees)
    oclock_angles = {
        12: 0,
        1: 30,
        2: 60,
        3: 90,
        4: 120,
        5: 150,
        6: 180,
        7: 210,
        8: 240,
        9: 270,
        10: 300,
        11: 330
    }
    return oclock_angles.get(oclock, None)

# Function to calculate if the turn is left or right
def calculate_turn(previous_direction, current_direction):
    turn_diff = (current_direction - previous_direction) % 360
    if turn_diff == 0:
        return "continue straight"
    elif turn_diff == 90 or turn_diff == 270:
        if turn_diff == 90:
            return "turn right"
        else:
            return "turn left"
    else:
        return "turn around"

# Function to create human-readable direction from walk instructions
def interpret_directions(instructions):
    current_direction = 0  # 12 o'clock direction
    direction_phrases = []
    
    for instruction in instructions:
        turn, distance = instruction
        new_direction = oclock_to_degrees(turn)
        
        if new_direction is None:
            continue
        
        turn_action = calculate_turn(current_direction, new_direction)
        
        if turn_action == "continue straight":
            direction_phrases.append(f"walk down the hallway {distance}ft")
        elif turn_action == "turn right":
            direction_phrases.append(f"take the hallway on your right, walk {distance}ft")
        elif turn_action == "turn left":
            direction_phrases.append(f"take the hallway on your left, walk {distance}ft")
        current_direction = new_direction
    
    return direction_phrases

# Example usage
instructions = [(3, 200), (3, 100)]  # (turn_direction, distance)
directions = interpret_directions(instructions)

for direction in directions:
    print(direction)
