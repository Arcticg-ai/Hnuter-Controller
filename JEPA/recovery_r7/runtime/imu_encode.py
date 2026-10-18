def encode(message,receive_ns):
 relative=int(message.accelerometer_timestamp_relative)
 if relative==2147483647 or message.accelerometer_clipping:return None
 return {'receive_monotonic_ns':int(receive_ns),'source_message_timestamp_us':int(message.timestamp),'source_accelerometer_sample_us':int(message.timestamp)+relative,'source_publication_timestamp_us':None,'source_timestamp_role':'gyro sample timestamp; accelerometer sample uses relative offset','specific_force_frd_m_s2':[float(x) for x in message.accelerometer_m_s2],'accelerometer_integral_dt_us':int(message.accelerometer_integral_dt),'accel_calibration_count':int(message.accel_calibration_count),'measurement_kind':'sensor_combined_calibrated_not_estimator_bias_corrected'}
