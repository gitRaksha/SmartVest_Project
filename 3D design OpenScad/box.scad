// Dimensions (adjusted for your components)
stm32_length = 100;   // PCB length
stm32_width = 67;     // PCB width
stm32_height = 15;    // Include pins (3mm PCB + 12mm pins)
wall_thickness = 2;    // Enclosure wall thickness
lid_height = 5;        // Lid thickness

// Main box outer dimensions
box_length = stm32_length + 10;  // Extra space
box_width = stm32_width + 10;    
box_height = stm32_height + lid_height + 5;  // Sensor clearance

// MQ7 sensor cutout
mq7_diameter = 17;  
mq7_height = 10;     

// Main box (hollow)
module box() {
    difference() {
        // Outer cube
        cube([box_length, box_width, box_height]);
        
        // Inner hollow (walls)
        translate([wall_thickness, wall_thickness, wall_thickness]) {
            cube([
                box_length - 2*wall_thickness,
                box_width - 2*wall_thickness,
                box_height + 1
            ]);
        }
        
        // Component holes
        holes();
    }
}

// Lid (snap-fit)
module lid() {
    difference() {
        cube([box_length, box_width, lid_height]);
        // Lip for fitting
        translate([wall_thickness, wall_thickness, -1]) {
            cube([
                box_length - 2*wall_thickness,
                box_width - 2*wall_thickness,
                lid_height + 2
            ]);
        }
    }
}

// Holes for components
module holes() {
    // LoRa antenna hole (side)
    translate([box_length - 5, box_width/2, box_height - 10]) {
        rotate([90, 0, 0]) {
            cylinder(h=wall_thickness + 2, d=6, center=true);
        }
    }
    
    // USB port hole (front)
    translate([15, -1, 10]) {
        cube([12, wall_thickness + 2, 8]);
    }
    
    // MQ7 sensor hole (top, circular)
    translate([box_length - 20, box_width - 10, box_height - mq7_height - 1]) {
        cylinder(h=mq7_height + 2, d=mq7_diameter);
    }
    
    // Slotted side vents (elongated)
    for (i = [0:4]) {
        // Left side
        translate([-1, 7 + i*15, box_height - 15]) {
            cube([wall_thickness + 2, 2, 9]);  // 10mm long, 3mm tall
        }
        // Right side
        translate([box_length - wall_thickness - 1, 7 + i*15, box_height - 15]) {
            cube([wall_thickness + 2, 2, 9]);
        }
    }
}

// Render box + lid (closed)
difference() {
    union() {
        box();
        translate([0, 0, box_height - lid_height]) lid();  // Lid in place
    }
    // Uncomment to debug cuts:
    // #holes();
}