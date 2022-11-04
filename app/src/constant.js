const BaudRateList = () => {
    return [ 9600, 14400, 19200, 38400, 57600, 115200 ]
}
const Tabs = () => {
    return ["connection", "servo", "leg", "actions", "sensors", "config", "serial"]
}
const UltrasonicTitle = () => {
    return [
        { id: 'front', title: 'Front' },
        { id: 'front_right', title: 'Front Right' },
        { id: 'right', title: 'Right' },
        { id: 'front_left', title: 'Front Left' },
        { id: 'left', title: 'Left' },
    ]
}
const LegTitle = () => {
    return [
        { id: 0, title: 'Front Right' },
        { id: 1, title: 'Rear Right' },
        { id: 2, title: 'Front Left' },
        { id: 3, title: 'Rear Left' },
    ]
}
const ConfigTitle = () => {
    return [
        { 
            title: 'Dimmension',
            id: 'dimmension',
            config: [
                { id: 'coxa_len', title: 'Coxa Len' },
                { id: 'femur_len', title: 'Femur Len' },
                { id: 'tibia_len', title: 'Tibia Len' },
            ]     
        }, { 
            title: 'Movement',
            id: 'movement',
            config: [
                { id: 'z_base', title: 'Z Base' },
                { id: 'z_stand', title: 'Z Stand' },
                { id: 'z_up', title: 'Z Up' },
                { id: 'x_base', title: 'X Base' },
                { id: 'y_base', title: 'Y Base' },
                { id: 'y_step', title: 'Y Step' },
            ]     
        }, { 
            title: 'Speed',
            id: 'speed',
            config: [
                { id: 'stand_seat_speed', title: 'Stand Seat Speed' },
                { id: 'leg_move_speed', title: 'Leg Move Speed' },
                { id: 'body_move_speed', title: 'Body Move Speed' },
                { id: 'spot_turn_speed', title: 'Spot Turn Speed' },
            ]
        }, { 
            title: 'Servo Calibration',
            id: 'servo_calib',
            type: 'table',
            rows: [
                { id: 'front_right', title: 'Front Right', },
                { id: 'rear_right', title: 'Rear Right', },
                { id: 'front_left', title: 'Front Left', },
                { id: 'front_right', title: 'Front Left', },
            ],
            cols: [
                { id: 'coxa', title: 'Coxa', },
                { id: 'femur', title: 'Femur', },
                { id: 'tibia', title: 'Tibia', },
            ]
        },
    ]
}
const UltrasonicUpdateInterval = () => {
    return [500, 1000, 2000, 3000, 5000]
}
module.exports = {
    BaudRateList,
    Tabs,
    UltrasonicTitle,
    LegTitle,
    ConfigTitle,
    UltrasonicUpdateInterval,
}