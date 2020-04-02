import csv
import re
import argparse
import sys
#import pylab

FLAGS = None

def write_list_of_dict(file,item_list):
    with open(file, 'wb') as f:
        w = csv.DictWriter(f, item_list[0].keys())
        w.writeheader()
        w.writerows(item_list)


def write_list_of_list(file,item_list):
    with open(file, 'wb') as f:
        w = csv.writer(f)
        #w.writeheader()
        w.writerows(item_list)


def main():
    #pattern='^.*\['+FLAGS.field_name+'\](.*)\n'
    #r'^.*\[state_cov\](.*)\n'
    patterns = []

    pattern0 = {}
    pattern0['field_name'] = "create_viofrontend_time"
    pattern0['pattern'] = re.compile('^.*create-viofrontend process time :(.*) FPS.*\n')
    pattern0['result'] = []
    patterns.append(pattern0)

    pattern0 = {}
    pattern0['field_name'] = "create_viofrontend_time1"
    pattern0['pattern'] = re.compile('^.*create-viofrontend time :(.*) FPS.*\n')
    pattern0['result'] = []
    patterns.append(pattern0)


    pattern0 = {}
    pattern0['field_name'] = "frontend_process_time"
    pattern0['pattern'] = re.compile('^.*frontend process time:(.*)\n')
    pattern0['result'] = []
    patterns.append(pattern0)

    pattern0 = {}
    pattern0['field_name'] = "frontend_prepare_time"
    pattern0['pattern'] = re.compile('^.*frontend_prepare_time: (.*)\n')
    pattern0['result'] = []
    patterns.append(pattern0)

    pattern0 = {}
    pattern0['field_name'] = "frontend_create_pyrm_time"
    pattern0['pattern'] = re.compile('^.*frontend_create_pyrm_time: (.*)\n')
    pattern0['result'] = []
    patterns.append(pattern0)

    pattern0 = {}
    pattern0['field_name'] = "frontend_init_or_track_time"
    pattern0['pattern'] = re.compile('^.*frontend_init_or_track_time: (.*)\n')
    pattern0['result'] = []
    patterns.append(pattern0)

    pattern0 = {}
    pattern0['field_name'] = "frontend_init_time"
    pattern0['pattern'] = re.compile('^.*frontend_init_time: (.*)\n')
    pattern0['result'] = []
    patterns.append(pattern0)

    pattern0 = {}
    pattern0['field_name'] = "frontend_track_time"
    pattern0['pattern'] = re.compile('^.*frontend_track_time: (.*)\n')
    pattern0['result'] = []
    patterns.append(pattern0)

    pattern0 = {}
    pattern0['field_name'] = "frontend_track_track_time"
    pattern0['pattern'] = re.compile('^.*frontend_track_track_time: (.*)\n')
    pattern0['result'] = []
    patterns.append(pattern0)

    pattern0 = {}
    pattern0['field_name'] = "frontend_track_stereo_match_time"
    pattern0['pattern'] = re.compile('^.*frontend_track_stereo_match_time: (.*)\n')
    pattern0['result'] = []
    patterns.append(pattern0)

    pattern0 = {}
    pattern0['field_name'] = "frontend_track_ransac_time"
    pattern0['pattern'] = re.compile('^.*frontend_track_ransac_time: (.*)\n')
    pattern0['result'] = []
    patterns.append(pattern0)

    pattern0 = {}
    pattern0['field_name'] = "frontend_add_features"
    pattern0['pattern'] = re.compile('^.*frontend_add_features: (.*)\n')
    pattern0['result'] = []
    patterns.append(pattern0)

    pattern0 = {}
    pattern0['field_name'] = "frontend_add_detect_features_time"
    pattern0['pattern'] = re.compile('^.*frontend_add_detect_features_time: (.*)\n')
    pattern0['result'] = []
    patterns.append(pattern0)

    pattern0 = {}
    pattern0['field_name'] = "frontend_add_or_init_detect_features_time"
    pattern0['pattern'] = re.compile('^.*DetectFASTFeature  detect_num: (.*) detect_time: (.*) descriptor_time: (.*)\n')
    pattern0['result'] = []
    patterns.append(pattern0)

    

    pattern0 = {}
    pattern0['field_name'] = "frontend_add_stereo_match_time"
    pattern0['pattern'] = re.compile('^.*frontend_add_stereo_match_time: (.*)\n')
    pattern0['result'] = []
    patterns.append(pattern0)

    pattern0 = {}
    pattern0['field_name'] = "frontend_add_add_time"
    pattern0['pattern'] = re.compile('^.*frontend_add_add_time: (.*)\n')
    pattern0['result'] = []
    patterns.append(pattern0)


    pattern0 = {}
    pattern0['field_name'] = "frontend_prune_grid_features"
    pattern0['pattern'] = re.compile('^.*frontend_prune_grid_features: (.*)\n')
    pattern0['result'] = []
    patterns.append(pattern0)

    pattern0 = {}
    pattern0['field_name'] = "frontend_updatefeaturelife_time"
    pattern0['pattern'] = re.compile('^.*frontend_updatefeaturelife_time: (.*)\n')
    pattern0['result'] = []
    patterns.append(pattern0)

    pattern0 = {}
    pattern0['field_name'] = "gnss_process_time"
    pattern0['pattern'] = re.compile('^.*GNSSFusion time:(.*)/(.*)\n')
    pattern0['result'] = []
    patterns.append(pattern0)

    pattern1 = {}
    pattern1['field_name'] = "backend_process_time_exclude_gnss"
    pattern1['pattern'] = re.compile('^.*backenddddddd process time in backend:(.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_process_time"
    pattern1['pattern'] = re.compile('^.*backend process time in backend:(.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_imu_processing_time"
    pattern1['pattern'] = re.compile('^.*IMU processing time: (.*)/.*\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_state_augmentation_time"
    pattern1['pattern'] = re.compile('^.*State augmentation time: (.*)/.*\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_add_observations_time"
    pattern1['pattern'] = re.compile('^.*Add observations time: (.*)/.*\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_remove_lost_features_time"
    pattern1['pattern'] = re.compile('^.*Remove lost features time: (.*)/.*\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_remove_lost_features_features_init"
    pattern1['pattern'] = re.compile('^.*backend_removeLostFeatures_features_init: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_remove_lost_features_feature_jacobian"
    pattern1['pattern'] = re.compile('^.*backend_removeLostFeatures_feature_jacobian: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_remove_lost_features_measureupdate"
    pattern1['pattern'] = re.compile('^.*backend_removeLostFeatures_measureupdate: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_measureupdate_QR"
    pattern1['pattern'] = re.compile('^.*backend_mu_QR: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_measureupdate_kalmangain"
    pattern1['pattern'] = re.compile('^.*backend_mu_kalmangain: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_mu_kalmangain_size"
    pattern1['pattern'] = re.compile('^.*backend_mu_kalmangain_size: (.*),.*\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_mu_kalmangain_S"
    pattern1['pattern'] = re.compile('^.*backend_mu_kalmangain_S: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_mu_kalmangain_K_transpose"
    pattern1['pattern'] = re.compile('^.*backend_mu_kalmangain_K_transpose: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_mu_kalmangain_delta_x"
    pattern1['pattern'] = re.compile('^.*backend_mu_kalmangain_delta_x: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)





    pattern1 = {}
    pattern1['field_name'] = "backend_measureupdate_total_state_cov_update"
    pattern1['pattern'] = re.compile('^.*backend_mu_total_state_cov_update: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_remove_camera_states_time"
    pattern1['pattern'] = re.compile('^.*Remove camera states time: (.*)/.*\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_remove_camera_states_features_init"
    pattern1['pattern'] = re.compile('^.*backend_pruneCamStateBuffer_features_init: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_remove_camera_states_feature_jacobian"
    pattern1['pattern'] = re.compile('^.*backend_pruneCamStateBuffer_feature_jacobian: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_feature_jacobian_matrix"
    pattern1['pattern'] = re.compile('^.*backend_pruneCamStateBuffer_feature_jacobian_matrix: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_feature_jacobian_matrix_H_xj_rows"
    pattern1['pattern'] = re.compile('^.*backend_pruneCamStateBuffer_feature_jacobian_matrix_H_xj_rows: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_feature_jacobian_matrix_H_xj_cols"
    pattern1['pattern'] = re.compile('^.*backend_pruneCamStateBuffer_feature_jacobian_matrix_H_xj_cols: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_feature_jacobian_matrix_H_fj_rows"
    pattern1['pattern'] = re.compile('^.*backend_pruneCamStateBuffer_feature_jacobian_matrix_H_fj_rows: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "backend_feature_jacobian_matrix_H_fj_cols"
    pattern1['pattern'] = re.compile('^.*backend_pruneCamStateBuffer_feature_jacobian_matrix_H_fj_cols: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)


    pattern1 = {}
    pattern1['field_name'] = "backend_feature_jacobian_svd"
    pattern1['pattern'] = re.compile('^.*backend_pruneCamStateBuffer_feature_jacobian_svd: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)


    pattern1 = {}
    pattern1['field_name'] = "backend_remove_camera_states_measureupdate"
    pattern1['pattern'] = re.compile('^.*backend_pruneCamStateBuffer_measureupdate: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "t_feature_triangulate"
    pattern1['pattern'] = re.compile('^.*t_feature_triangulate_time: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "t_optimization"
    pattern1['pattern'] = re.compile('^.*t_optimization_time: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "t_optimization_prepare"
    pattern1['pattern'] = re.compile('^.*t_optimization_prepare: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "t_optimization_optimize"
    pattern1['pattern'] = re.compile('^.*t_optimization_optimize: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "t_marginalization"
    pattern1['pattern'] = re.compile('^.*t_marginalization: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "t_marginalization_create_constraint"
    pattern1['pattern'] = re.compile('^.*t_marginalization_create_constraint: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "t_marginalization_pre_margin"
    pattern1['pattern'] = re.compile('^.*t_marginalization_pre_margin: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "t_marginalization_margin"
    pattern1['pattern'] = re.compile('^.*t_marginalization_margin: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "t_marginalization_after_margin"
    pattern1['pattern'] = re.compile('^.*t_marginalization_after_margin: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)


    pattern1 = {}
    pattern1['field_name'] = "t_post_processing"
    pattern1['pattern'] = re.compile('^.*t_post_processing_time: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)


    pattern1 = {}
    pattern1['field_name'] = "UpdateLocalMap_time"
    pattern1['pattern'] = re.compile('^.*UpdateLocalMap_time: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)


    pattern1 = {}
    pattern1['field_name'] = "UpdateLocalKeyFrames_time"
    pattern1['pattern'] = re.compile('^.*UpdateLocalKeyFrames_time: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "UpdateLocalKeyFrames_step1"
    pattern1['pattern'] = re.compile('^.*UpdateLocalKeyFrames_step1: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "UpdateLocalKeyFrames_step2"
    pattern1['pattern'] = re.compile('^.*UpdateLocalKeyFrames_step2: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "UpdateLocalKeyFrames_step3"
    pattern1['pattern'] = re.compile('^.*UpdateLocalKeyFrames_step3: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)



    pattern1 = {}
    pattern1['field_name'] = "UpdateLocalPoints_time"
    pattern1['pattern'] = re.compile('^.*UpdateLocalPoints_time: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)


    pattern1 = {}
    pattern1['field_name'] = "SearchLocalPoints_time"
    pattern1['pattern'] = re.compile('^.*SearchLocalPoints_time: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "SearchLocalPoints_step1"
    pattern1['pattern'] = re.compile('^.*SearchLocalPoints_step1: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "SearchLocalPoints_step1_size"
    pattern1['pattern'] = re.compile('^.*SearchLocalPoints_step1_size: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "SearchLocalPoints_step2p1"
    pattern1['pattern'] = re.compile('^.*SearchLocalPoints_step2p1: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "SearchLocalPoints_step2p2"
    pattern1['pattern'] = re.compile('^.*SearchLocalPoints_step2p2: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "SearchLocalPoints_step2"
    pattern1['pattern'] = re.compile('^.*SearchLocalPoints_step2: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "SearchLocalPoints_step2_size"
    pattern1['pattern'] = re.compile('^.*SearchLocalPoints_step2_size: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)






    pattern1 = {}
    pattern1['field_name'] = "SearchLocalPoints_step1"
    pattern1['pattern'] = re.compile('^.*SearchLocalPoints_step1: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "SearchLocalPoints_step2"
    pattern1['pattern'] = re.compile('^.*SearchLocalPoints_step2: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "SearchByProjection_vpMapPoints_num"
    pattern1['pattern'] = re.compile('^.*SearchByProjection_vpMapPoints_num: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "SearchByProjection_step1"
    pattern1['pattern'] = re.compile('^.*SearchByProjection_step1: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "SearchLocalPoints_step3"
    pattern1['pattern'] = re.compile('^.*SearchLocalPoints_step3: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)


    pattern1 = {}
    pattern1['field_name'] = "PoseOptimization_time"
    pattern1['pattern'] = re.compile('^.*PoseOptimization_time: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "PoseOptimization_num"
    pattern1['pattern'] = re.compile('^.*PoseOptimization_num: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)


    pattern1 = {}
    pattern1['field_name'] = "UpdateMapPointsStatistics_time"
    pattern1['pattern'] = re.compile('^.*UpdateMapPointsStatistics_time: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "ORB_TrackLocalMap_time"
    pattern1['pattern'] = re.compile('^.*ORB_TrackLocalMap_time: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "ORB_TrackLocalMap_status"
    pattern1['pattern'] = re.compile('^.*ORB_TrackLocalMap_status: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)






    #  099 Hessian rows and cols =307,307
    pattern1 = {}
    pattern1['field_name'] = "ceres-Hessian-rows-and-cols"
    pattern1['pattern'] = re.compile('^.*Hessian rows and cols =(.*),(.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)


    #  100 Jacobian rows and cols =3292,307
    pattern1 = {}
    pattern1['field_name'] = "ceres-Jacobian-rows-and-cols"
    pattern1['pattern'] = re.compile('^.*Jacobian rows and cols =(.*),(.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)


    #  101 SchurTime=      0.0020679740000559831969
    pattern1 = {}
    pattern1['field_name'] = "ceres-SchurTime"
    pattern1['pattern'] = re.compile('^.*SchurTime=      (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    #  102 S matrix rows and cols = 120, 120
    pattern1 = {}
    pattern1['field_name'] = "ceres-S-matrix-rows-and-cols"
    pattern1['pattern'] = re.compile('^.*S matrix rows and cols = (.*), (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    #  103 CholeskyTime=        0.0001225949999934528023
    pattern1 = {}
    pattern1['field_name'] = "ceres-CholeskyTime"
    pattern1['pattern'] = re.compile('^.*CholeskyTime=        (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    #  104 BackSubstitudeTime=        0.00014929200006008613855
    pattern1 = {}
    pattern1['field_name'] = "ceres-BackSubstitudeTime"
    pattern1['pattern'] = re.compile('^.*BackSubstitudeTime=        (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    #  105 ComputeStepTime=0.0025717039998198742978
    pattern1 = {}
    pattern1['field_name'] = "ceres-ComputeStepTime"
    pattern1['pattern'] = re.compile('^.*ComputeStepTime=(.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    #  106 CostTime=       0.00041226699977414682508
    pattern1 = {}
    pattern1['field_name'] = "ceres-CostTime"
    pattern1['pattern'] = re.compile('^.*CostTime=       (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    #  107 JacobianTime=   0.0014914690000296104699
    pattern1 = {}
    pattern1['field_name'] = "ceres-JacobianTime"
    pattern1['pattern'] = re.compile('^.*JacobianTime=   (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)


    pattern1 = {}
    pattern1['field_name'] = "ceres-NUM_ITERATIONS"
    pattern1['pattern'] = re.compile('^.*NUM_ITERATIONS: (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)


    pattern1 = {}
    pattern1['field_name'] = "marginalize_A_size"
    pattern1['pattern'] = re.compile('^.*marginalize A size: (.*) size: (.*),(.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "marginalize_m_n_size"
    pattern1['pattern'] = re.compile('^.*marginalize m:(.*) n:(.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "ceres-Minimizer_iterations"
    pattern1['pattern'] = re.compile('^.*Minimizer iterations                        (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "ceres-Successful_steps"
    pattern1['pattern'] = re.compile('^.*Successful steps                            (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)

    pattern1 = {}
    pattern1['field_name'] = "ceres-Unsuccessful_steps"
    pattern1['pattern'] = re.compile('^.*Unsuccessful steps                          (.*)\n')
    pattern1['result'] = []
    patterns.append(pattern1)



    '''
    pattern2 = {}
    pattern2['field_name'] = "tx1-frontend_process_time"
    pattern2['pattern'] = re.compile('^.*total image process: (.*)\n')
    pattern2['result'] = []
    patterns.append(pattern2)

    pattern3 = {}
    pattern3['field_name'] = "tx1-backend_process_time"
    pattern3['pattern'] = re.compile('^.*IMU processing time: (.*)/(.*)\n')
    pattern3['result'] = []
    patterns.append(pattern3)
    '''



    with open(FLAGS.logfile, "rb") as f:
      line = True
      item = {}
      while line:
        line = f.readline()
        for pattern0 in patterns:
          regMatch = pattern0['pattern'].match(line)
          if regMatch:
            if pattern0['field_name'] == "tx1-backend_process_time":
              matchval = float(regMatch.group(1))
              val2 = float(regMatch.group(2))
              pattern0['result'].append([matchval*1.0/val2])
            if pattern0['field_name'] == "gnss_process_time":
              val1 = float(regMatch.group(1))
              val2 = float(regMatch.group(2))
              pattern0['result'].append([val1])

              for pattern0 in patterns:
                if pattern0['field_name'] == "backend_process_time_exclude_gnss":
                  pattern0['result'].append([val1*1.0/val2 - val1])
            else:
              #print(regMatch.groups())
              rows = []
              for val in regMatch.groups():
                matchval = float(val)
                rows.append(matchval)
              pattern0['result'].append(rows)
              '''
              matchval = float(regMatch.group(1))
              pattern0['result'].append([matchval])
              '''


    # write
    for pattern0 in patterns:
        if len(pattern0['result']) > 0 :
            write_list_of_list(pattern0['field_name']+".csv", pattern0['result'])

    #print(len(val_list))
    #pylab.figure()
    #pylab.subplot(111)
    #pylab.plot(range(len(val_list[:1000])), val_list[:1000])
    #pylab.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    #parser.add_argument('-T','--state_cov', type=str, default='state_cov.csv', help='file for storing input data')
    #parser.add_argument('-F','--field_name', type=str, default='state_cov', help='file for storing input data')
    #parser.add_argument('-L','--logfile', type=str, default='/home/andy/Desktop/vio_eval/2019-1211/tx1/Localization_2019-12-11_12-08-22/dragonflyvio_server.INFO', help='Directory for storing input data')
    parser.add_argument('-L','--logfile', type=str, default='/home/andy/Desktop/vio_eval/2019-1211/pc/Localization_2019-12-11_19-51-50/dragonflyvio_server.INFO', help='Directory for storing input data')

    FLAGS, unparsed = parser.parse_known_args()
    main()
