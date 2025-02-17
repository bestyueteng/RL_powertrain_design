function [motor_Data,scales] = get_motor_scale_type(m_block_name)
   motor_type = get_param(m_block_name, 'motortype'); 
   if strcmp(motor_type, 'Permanent magnet') 
       % motor_number = get_param(m_block_name, 'motornumberPM'); 
       % motor_Data{end+1} = ['MC_PM' motor_number]; 
       % scales(end+1) = get_param(m_block_name, 'scale_EM'); 
       motor_number = get_param(m_block_name, 'motornumberPM');
       numbers = regexp(motor_number, '\d', 'match'); 
       motor_number = [numbers{:}];
       motor_Data = ['MC_PM' motor_number];
       scales = get_param(m_block_name, 'scale_EM');
   else 
       motor_number = get_param(m_block_name, 'motornumberAC');
       numbers = regexp(motor_number, '\d', 'match'); 
       motor_number = [numbers{:}];
       motor_Data = ['MC_AC' motor_number];
       scales = get_param(m_block_name, 'scale_EM');
   end
end