function dataLogger(robot)
    filename = robot.model.name;
    log = fopen(filename, 'a');
    
    joints     = robot.model.getpos();
    endefector = robot.model.fkine(joints);
    endefector = endefector(1:3,4);
    
    fprintf(log, 'Current Joints: %.4f %.4f %.4f %.4f %.4f %.4f \n', joints);
    fprintf(log, 'Endeffector Pose: %.4f %.4f %.4f %.4f \n', endefector);

    fclose(log);
end

