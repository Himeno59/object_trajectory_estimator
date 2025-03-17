#include "ros/ros.h"
#include "bouncing_ball_estimator/SetRLSMatrix.h"
#include <vector>

int main(int argc, char **argv) {
    ros::init(argc, argv, "set_rls_matrix_client");
    ros::NodeHandle nh;

    // コマンドライン引数の数を確認
    if (argc < 3) {
        ROS_ERROR("Usage: set_rls_matrix_client <rows> <cols> <data1> <data2> ... <dataN>");
        return 1;
    }

    // 行数と列数をコマンドライン引数から取得
    int rows = atoi(argv[1]);
    int cols = atoi(argv[2]);

    // データ要素数の確認
    if (argc != 3 + rows * cols) {
        ROS_ERROR("Invalid number of data elements. Expected %d elements.", rows * cols);
        return 1;
    }

    // サービスクライアントの作成
    ros::ServiceClient client = nh.serviceClient<bouncing_ball_estimator::SetRLSMatrix>("/BouncingBallEstimator/set_rls_matrix");

    // サービスリクエストのオブジェクトを作成
    bouncing_ball_estimator::SetRLSMatrix srv;

    // リクエストに行数と列数を設定
    srv.request.rows = rows;
    srv.request.cols = cols;

    // 行列データを設定
    for (int i = 3; i < argc; ++i) {
        srv.request.data.push_back(atof(argv[i])); // コマンドライン引数を浮動小数点数に変換
    }

    // サービスの呼び出し
    if (client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("Matrix set successfully.");
        } else {
            ROS_WARN("Failed to set matrix.");
        }
    } else {
        ROS_ERROR("Failed to call service set_rls_matrix.");
    }

    return 0;
}
