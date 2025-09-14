// LID ONLY - With all required cutouts
$fn = 50; // Smoothness

// Dimensions (must match main box)
lid_length = 110;   // box_length = stm32_length + 10
lid_width = 77;     // box_width = stm32_width + 10
lid_height = 5;     // Lid thickness
wall_thickness = 2; 

// MQ7 sensor cutout
mq7_diameter = 17;  
mq7_x = 90;         // Position from left (box_length - 20)
mq7_y = 67;         // Position from front (box_width - 10)

// LoRa antenna hole
antenna_diameter = 6;
antenna_x = 105;    // box_length - 5
antenna_y = lid_width/2;

// Ventilation slots (vertical)
module ventilation_slots() {
    slot_length = 9;  // Height (vertical)
    slot_width = 2;   // Thickness
    for (i = [0:4]) {
        // Left side
        translate([-1, 7 + i*15, lid_height/2]) {
            cube([wall_thickness + 2, slot_width, slot_length], center=true);
        }
        // Right side
        translate([lid_length - wall_thickness - 1, 7 + i*15, lid_height/2]) {
            cube([wall_thickness + 2, slot_width, slot_length], center=true);
        }
    }
}

// Main lid with holes
difference() {
    // Base lid
    cube([lid_length, lid_width, lid_height]);
    
    // MQ7 sensor hole (top)
    translate([mq7_x, mq7_y, -1]) {
        cylinder(h=lid_height + 2, d=mq7_diameter);
    }
    
    // LoRa antenna hole (side)
    translate([antenna_x, antenna_y, lid_height/2]) {
        rotate([90, 0, 0]) {
            cylinder(h=wall_thickness + 2, d=antenna_diameter, center=true);
        }
    }
    
  
}