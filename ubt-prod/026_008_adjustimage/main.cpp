#include <iostream>
#include <filesystem>
#include <fmt/format.h>
#include <opencv2/opencv.hpp>

int main(int argc, char *argv[]) {
    /****************************/
    /* 3頂点と長さ(2つ)を求める */
    /****************************/
    /* ベース画像読み込み */
    cv::Mat4b stdposimg = cv::imread("/media/sf_timelapse_sakura/pictures/026_008_makers/base_IMG_20230326_101240_1011.png", cv::IMREAD_UNCHANGED);
    if(stdposimg.empty()) {
        std::cout << "画像の読み込みに失敗!! base_IMG_20230326_101240_1011.png" << std::endl;
        return 1;
    }
    std::cout << fmt::format("ベース画像(base_IMG_20230326_101240_1011.png) サイズ={}x{} channels: {}", stdposimg.cols, stdposimg.rows, stdposimg.channels()) << std::endl;

    /* 3角形に見えんけど、3角形のつもり */
    /*                                  */
    /*              top                 */
    /*              △                  */
    /*             /  ＼                */
    /*            /     ＼t2r           */
    /*           /        ＼            */
    /*          /          △ right     */
    /*         /       ////             */
    /*        /    ////                 */
    /*       △////    b2r              */
    /*       bottom                     */
    /*                                  */
    /*                                  */

    /* 3頂点を取得 */
    cv::Point2f basetop  = cv::Point2f(0, stdposimg.cols);
    cv::Point2f basebtm  = cv::Point2f(0, 0);
    cv::Point2f baseright= cv::Point2f(0, 0);
    for(int y = 0; y < stdposimg.rows; y++) {
        for(int x = 0; x < stdposimg.cols; x++) {
            int B = stdposimg.data[ y*stdposimg.step + x*stdposimg.elemSize() + 0 ];
            int G = stdposimg.data[ y*stdposimg.step + x*stdposimg.elemSize() + 1 ];
            int R = stdposimg.data[ y*stdposimg.step + x*stdposimg.elemSize() + 2 ];
            int A = stdposimg.data[ y*stdposimg.step + x*stdposimg.elemSize() + 3 ];
            if(R == 255) {
                // std::cout << fmt::format("{}x{} B:{} G:{} R:{} A:{}", x, y, B, G, R, A) << std::endl;
                /* 頂点(上) */
                if(y < basetop.y)
                    basetop  = cv::Point2f(x, y);
                /* 頂点(下) */
                if(y > basebtm.y)
                    basebtm  = cv::Point2f(x, y);
                /* 頂点(右) */
                if(x > baseright.x)
                    baseright= cv::Point2f(x, y);
            }
        }
    }
    std::cout << fmt::format("ベースtop(\t{}\t, \t{}\t) ベースbottom(\t{}\t, \t{}\t) ベースright(\t{}\t, \t{}\t)", basetop.x,basetop.y, basebtm.x,basebtm.y, baseright.x,baseright.y) << std::endl;

    /* 3頂点から長さ取得(top2right, bottom2right) */
    int baselent2r = std::sqrt((double)(baseright.x-basetop.x)*(baseright.x-basetop.x) + (baseright.y-basetop.y)*(baseright.y-basetop.y));
    int baselenb2r = std::sqrt((double)(baseright.x-basebtm.x)*(baseright.x-basebtm.x) + (baseright.y-basebtm.y)*(baseright.y-basebtm.y));
    std::cout << fmt::format("ベース長さ(\t{}\t, \t{}\t) ", baselent2r, baselenb2r) << std::endl;

    /******************/
    /* 画像合わせ処理 */
    /******************/
    int lpct = 0;
    namespace fs = std::filesystem;
    fs::remove_all("/media/sf_timelapse_sakura/pictures/026_009_imgmove");
    fs::remove_all("/media/sf_timelapse_sakura/pictures/026_009_makermove");
    fs::remove_all("/media/sf_timelapse_sakura/pictures/026_010_imgcomplete");
    fs::remove_all("/media/sf_timelapse_sakura/pictures/026_010_makercomplete");
    fs::remove_all("/media/sf_timelapse_sakura/pictures/026_011_makercomplete");
    for(const fs::directory_entry &file : fs::directory_iterator("/media/sf_timelapse_sakura/pictures/026_006_Thinout")) {
        std::cout << fmt::format("----------------------  ---------------------- lpct={}", lpct++) << std::endl;
        /* マーカー画像のパスを生成 */
        std::string makerfilefullpath = file.path().parent_path().parent_path().string() + "/026_008_makers/" + file.path().stem().string() + ".png";
        std::cout << makerfilefullpath << std::endl;

        /* マーカー画像を開く */
        cv::Mat4b makerimg = cv::imread(makerfilefullpath, cv::IMREAD_UNCHANGED);
        if(makerimg.empty()) {
            std::cout << fmt::format("マーカー画像の読み込みに失敗!! {}", makerfilefullpath) << std::endl;
            return 1;
        }
        std::cout << fmt::format("マーカー画像 サイズ={}x{} channels: {}", makerimg.cols, makerimg.rows, makerimg.channels()) << std::endl;

        /* 基準線(=マーカー線)              */
        /*                                  */
        /*          maker-top               */
        /*              △                  */
        /*             /                    */
        /*            /                     */
        /*           /                      */
        /*          /                       */
        /*         /                        */
        /*        /                         */
        /*       △                         */
        /*     maker-btm                    */
        /*                                  */
        /* 基準線の座標を取得 */
        cv::Point2f makertop = cv::Point2f(0, makerimg.cols);
        cv::Point2f makerbtm = cv::Point2f(0, 0);
        for(int y = 0; y < makerimg.rows; y++) {
            for(int x = 0; x < makerimg.cols; x++) {
                int B = makerimg.data[ y*makerimg.step + x*makerimg.elemSize() + 0 ];
                int G = makerimg.data[ y*makerimg.step + x*makerimg.elemSize() + 1 ];
                int R = makerimg.data[ y*makerimg.step + x*makerimg.elemSize() + 2 ];
                int A = makerimg.data[ y*makerimg.step + x*makerimg.elemSize() + 3 ];
                if(R == 255) {
                    /* 頂点(上) */
                    if(y < makertop.y)
                        makertop  = cv::Point2f(x, y);
                    /* 頂点(下) */
                    if(y > makerbtm.y)
                        makerbtm  = cv::Point2f(x, y);
                }
            }
        }
        std::cout << fmt::format("{}\t maker-top(\t{}\t, \t{}\t) maker-btm(\t{}\t, \t{}\t)", makerfilefullpath, makertop.x,makertop.y, makerbtm.x,makerbtm.y) << std::endl;

        /* マーカー線をベース線上に移動           */
        /*                                        */
        /*           maker-top           base-top */
        /*               △                  △   */
        /*              /                 ／      */
        /*             /                ／        */
        /*            /               ／          */
        /*           /              ／            */
        /*          /             ／              */
        /*         /    ┌-----> △               */
        /*        △  --┘      base-btm          */
        /*      maker-btm                         */
        /*                                        */
        /* 移動量を求める */
        double movex = basebtm.x-makerbtm.x;
        double movey = basebtm.y-makerbtm.y;
        cv::Mat moveMatrix = (cv::Mat_<double>(2,3) << 1.0, 0.0, movex, 0.0, 1.0, movey);

        /* 平行移動を実行 */
        cv::Mat4b makermoveimg;
        cv::warpAffine(makerimg, makermoveimg, moveMatrix, makerimg.size(), cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        /* マーカー画像の平行移動を出力先に保存 */
        std::string makermovepath = file.path().parent_path().parent_path().string() + "/026_009_makermove/";
        fs::create_directories(makermovepath);
        std::string makermovefilefullpath = file.path().parent_path().parent_path().string() + "/026_009_makermove/" + file.path().stem().string() + ".png";
        cv::imwrite(makermovefilefullpath, makermoveimg);

        /* 移動後頂点を算出 */
        cv::Point2f makertopmove = cv::Point2f(makertop.x+movex, makertop.y+movey);
        cv::Point2f makerbtmmove = cv::Point2f(makerbtm.x+movex, makerbtm.y+movey);
        std::cout << fmt::format("000 平行移動後頂点 makertopmove(\t{}\t, \t{}\t) makerbtmmove(\t{}\t, \t{}\t)", makertopmove.x,makertopmove.y, makerbtmmove.x,makerbtmmove.y) << std::endl;

        /* 拡縮率を求める */
        double baselent2b = std::sqrt( (double)(basetop.x -basebtm.x )*(basetop.x -basebtm.x) + (basetop.y -basebtm.y)*(basetop.y -basebtm.y ) );
        double makerlent2b= std::sqrt( (double)(makertopmove.x-makerbtmmove.x)*(makertopmove.x-makerbtmmove.x) + (makertopmove.y-makerbtmmove.y)*(makertopmove.y-makerbtmmove.y) );
        double scale = (double)makerlent2b / baselent2b;
        /* マーカーの長さ(makerlen-t2r, makerlen-b2r)を求める */
        double makerlent2r = baselent2r * scale;
        double makerlenb2r = baselenb2r * scale;
        std::cout << fmt::format("111 拡縮率を求める scale(\t{}\t) t2b(base:\t{}\t, maker:\t{}\t) makerlen-t2r(\t{}\t) makerlen-b2r(\t{}\t)", scale, baselent2b,makerlent2b, makerlent2r, makerlenb2r) << std::endl;

        /* 回転/拡縮の中心点を求める */
        // https://memo.sugyan.com/entry/20090408/1239148436
        // https://qiita.com/UchiwaFuujinn/items/12cb4de120ac5af292f8
        // https://okwave.jp/qa/q8906319.html
        /*                                                     */
        /*          makermove-top                              */
        /*                ↑                                   */
        /*              △｜                                   */
        /*             /                                       */
        /*            /                                        */
        /*           /                                         */
        /*          /                                          */
        /*     ↑  /                                           */
        /*     ｜ /                                            */
        /*       △ makermove-btm                              */
        /*       ↑この点を点oに移動 : 移動量(offsetx,offsety) */
        /*                                          */
        /* 準備1.基準線を点Oに移動(移動量は保持っとく) */
        int offsetx = makerbtmmove.x;
        int offsety = makerbtmmove.y;
        cv::Point2f makertopmvoetrans = cv::Point2f(makertopmove.x-offsetx, makertopmove.y-offsety);/* pointbtm分のOffset */
        cv::Point2f makerbtmmovetrans = cv::Point2f(makerbtmmove.x-offsetx, makerbtmmove.y-offsety);/* pointbtmを点Oにする */
        std::cout << fmt::format("222 tmp移動量求め移動 offset(\t{}\t, \t{}\t) \nmaker-top-movetrans(\t{}\t, \t{}\t) maker-btm-movetrans(\t{}\t, \t{}\t)", offsetx,offsety, makertopmvoetrans.x,makertopmvoetrans.y, makerbtmmovetrans.x,makerbtmmovetrans.y) << std::endl;

        /*                                       */
        /*                                       */
        /*              △                       */
        /*             /        x軸上に回転移動  */
        /*            /   ⤵    移動量(-theta)   */
        /*           /      ⤵                   */
        /*          /         ⤵                 */
        /*         /           ⤵                */
        /*  bottom/                              */
        /*       △_______________△ top         */
        /*                                       */
        /* 準備2.基準線をy軸上になるように回転 */
        double tilt = (double)makertopmvoetrans.y / makertopmvoetrans.x;
        double theta = (makertopmvoetrans.x < 0) ? M_PI + std::atan(tilt) : std::atan(tilt);
        std::cout << fmt::format("333 tmp回転量を求める theta(\t{}\t)", theta) << std::endl;
        /* x′=xcos(θ)−ysin(θ) */
        /* y′=xsin(θ)+ycos(θ) */
        double makertopmvoetransrotx = makertopmvoetrans.x * std::cos(-theta) - makertopmvoetrans.y * std::sin(-theta);
        double makertopmvoetransroty = makertopmvoetrans.x * std::sin(-theta) + makertopmvoetrans.y * std::cos(-theta);
        cv::Point2f makertopmvoetransrot = cv::Point2f(makertopmvoetransrotx, makertopmvoetransroty);   /* x軸上に回転 */
        cv::Point2f makerbtmmovetransrot = makerbtmmovetrans;                                       /* 点Oのまま */
        std::cout << fmt::format("444 tmp回転 maker-top-movetransrot(\t{}\t, \t{}\t) maker-btm-movetransrot(\t{}\t, \t{}\t)", makertopmvoetransrot.x,makertopmvoetransrot.y, makerbtmmovetransrot.x,makerbtmmovetransrot.y) << std::endl;

        /*                                   */
        /*     bottom                 top    */
        /*       △___________________△     */
        /*         \                ／       */
        /*          \             ／         */
        /*           \          ／           */
        /*       b2r  \       ／  t2r        */
        /*             \    ／               */
        /*              \ ／                 */
        /*               △ <-こいつを求める */
        /*                                   */
        /* 準備3.(準備2の2座標)と(3頂点から求めた長さ)を使って、のこり1つの座標をもとめる */
        /* centerposmovetransrotx = (-t2r^2 + b2r^2 + top.x^2) / (2 * top.x) */
        /* centerposmovetransroty = sqrt(b2r^2 - centerposmovetransrotx^2) = sqrt(t2r^2 - makertopmvoetransrot.x^2 + 2*makertopmvoetransrot.x*centerposmovetransrotx - centerposmovetransrotx^2) */
        double centerposmovetransrotx = (-(makerlent2r*makerlent2r) + (makerlenb2r*makerlenb2r) + (makertopmvoetransrot.x*makertopmvoetransrot.x)) / (2 * makertopmvoetransrot.x);
        double centerposmovetransroty = std::sqrt((makerlenb2r*makerlenb2r) - (centerposmovetransrotx*centerposmovetransrotx));
        double centerposmovetransroty2= std::sqrt((makerlent2r*makerlent2r) - (makertopmvoetransrot.x*makertopmvoetransrot.x) + 2*makertopmvoetransrot.x*centerposmovetransrotx - (centerposmovetransrotx*centerposmovetransrotx));
        std::cout << fmt::format("555 (仮)中心座標算出 tmp(\t{}\t, \t{}\t, \t{}\t)", centerposmovetransrotx, centerposmovetransroty, centerposmovetransroty2) << std::endl;

        /* 準備4.回転を元に戻す */
        double centerposmovetransx = centerposmovetransrotx * std::cos(theta) - centerposmovetransroty * std::sin(theta);
        double centerposmovetransy = centerposmovetransrotx * std::sin(theta) + centerposmovetransroty * std::cos(theta);
        cv::Point2f centerposmovetrans = cv::Point2f(centerposmovetransx, centerposmovetransy);

        double makertopmvoetransx_r = makertopmvoetransrot.x * std::cos(theta) - makertopmvoetransrot.y * std::sin(theta);
        double makertopmvoetransy_r = makertopmvoetransrot.x * std::sin(theta) + makertopmvoetransrot.y * std::cos(theta);
        cv::Point2f makertopmvoetrans_r = cv::Point2f(makertopmvoetransx_r, makertopmvoetransy_r);
        cv::Point2f makerbtmmovetrans_r = makerbtmmovetransrot;                               /* 点Oのまま */
        std::cout << fmt::format("666 tmp回転戻し makertop(\t{}\t, \t{}\t) makerbtm(\t{}\t, \t{}\t) centerpos(\t{}\t, \t{}\t)", makertopmvoetrans_r.x,makertopmvoetrans_r.y, makerbtmmovetrans_r.x,makerbtmmovetrans_r.y, centerposmovetrans.x,centerposmovetrans.y) << std::endl;

        /* 準備5.移動を元に戻す */
        cv::Point2f centerposmove  = cv::Point2f(centerposmovetrans.x+offsetx,  centerposmovetrans.y+offsety);
        cv::Point2f makertopmove_r = cv::Point2f(makertopmvoetrans_r.x+offsetx, makertopmvoetrans_r.y+offsety);
        cv::Point2f makerbtmmove_r = cv::Point2f(makerbtmmovetrans_r.x+offsetx, makerbtmmovetrans_r.y+offsety);
        std::cout << fmt::format("777 tmp移動戻し makertop(\t{}\t, \t{}\t) makerbtm(\t{}\t, \t{}\t) centerposmove(\t{}\t, \t{}\t)", makertopmove_r.x,makertopmove_r.y, makerbtmmove_r.x,makerbtmmove_r.y, centerposmove.x,centerposmove.y) << std::endl;

        /* 拡縮量を求める */
//      double scale = (double)makerlent2b / baselent2b;    /* すでに算出済み */

        /* 回転量を求める(y軸とマーカー線から) */
        double rotatetilt = ((double)makertopmove.y-makerbtmmove.y) / (makertopmove.x-makerbtmmove.x);
        double rotaterad = std::atan(rotatetilt);
        double rotatedegree = (rotaterad < 0) ? (90) + rotaterad * (180/M_PI) : (90) - rotaterad * (180/M_PI);
        std::cout << fmt::format("888 回転量求める rotatedegree(\t{}\t) rotaterad(\t{}\t) rotatetilt(\t{}\t) makertopmove(\t{}\t, \t{}\t) makerbtm(\t{}\t, \t{}\t)", rotatedegree, rotaterad, rotatetilt, makertopmove.x,makertopmove.y, makerbtm.x,makerbtm.y) << std::endl;
        std::cout << fmt::format("888 材料揃った centerposmove(\t{}\t, \t{}\t) rotatedegree(\t{}\t) scale(\t{}\t)", centerposmove.x,centerposmove.y, rotatedegree, scale) << std::endl;

        /* マーカー画像は読込み済 */
//      cv::Mat4b makermoveimg = cv::imread(makermovefilefullpath, cv::IMREAD_UNCHANGED);
//      /* Target画像読込み(Open) */
//      cv::Mat4b targetimg = cv::imread(makerfilefullpath, cv::IMREAD_UNCHANGED);
//      if(targetimg.empty()) {
//          std::cout << fmt::format("Target画像の読み込みに失敗!! {}", makerfilefullpath) << std::endl;
//          return 1;
//      }
//      std::cout << fmt::format("Target画像 サイズ={}x{} channels: {}", targetimg.cols, targetimg.rows, targetimg.channels()) << std::endl;

        /* 回転&拡縮の行列取得 */
        cv::Mat rotscaleMatrix = cv::getRotationMatrix2D(centerposmove, -rotatedegree, 1/scale);
        /* 回転&拡縮 実行 */
        cv::Mat4b makermoverotscaleimg;
        cv::warpAffine(makermoveimg, makermoverotscaleimg, rotscaleMatrix, makermoveimg.size(), cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        /* 加工完了マーカー画像を出力先に保存 */
        std::string makercompletepath = file.path().parent_path().parent_path().string() + "/026_010_makercomplete/";
        fs::create_directories(makercompletepath);
        std::string makercompletefilefullpath = file.path().parent_path().parent_path().string() + "/026_010_makercomplete/" + file.path().stem().string() + ".png";
        cv::imwrite(makercompletefilefullpath, makermoverotscaleimg);
        // if(file.path().stem().string() == "IMG_20230324_072944_1005")
        //     int aaaaaaaa = 0;
        // double rotdelta   = (rotatedegree/10);
        // double scaledelta = 1.0;
        // for(int lpctaaa = 0; lpctaaa < 10; lpctaaa++, rotdelta+=(rotatedegree/10), scaledelta-=(scale/10)) {
        //     /* 回転&拡縮の行列取得 */
        //     cv::Mat rotscaleMatrix = cv::getRotationMatrix2D(centerposmove, -rotdelta, 1/scaledelta);
        //     /* 回転&拡縮 実行 */
        //     cv::Mat4b aaaaa;
        //     cv::warpAffine(makermoveimg, aaaaa, rotscaleMatrix, makermoveimg.size(), cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        //     /* 加工完了マーカー画像を出力先に保存 */
        //     std::string aaadir = file.path().parent_path().parent_path().string() + "/026_010_makercomplete/";
        //     fs::create_directories(aaadir);
        //     std::string aaafullpath = file.path().parent_path().parent_path().string() + "/026_010_makercomplete/" + file.path().stem().string() + fmt::format("___{}", lpctaaa) + ".png";
        //     cv::imwrite(aaafullpath, aaaaa);
        // }

        // {
        // cv::Mat image = cv::Mat::zeros(cv::Size(2560,1280), CV_8UC3);
        // cv::line(image, centerposmove, makertopmove_r, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        // cv::line(image, centerposmove, makerbtmmove_r, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
        // std::string makercompletepath = file.path().parent_path().parent_path().string() + "/026_011_makercomplete/";
        // fs::create_directories(makercompletepath);
        // std::string makercompletefilefullpath = file.path().parent_path().parent_path().string() + "/026_011_makercomplete/" + file.path().stem().string() + ".png";
        // cv::imwrite(makercompletefilefullpath, image);
        // }
    }

    return 0;
}

#if 0
cd ubt-prod/026_008_adjustimage
/usr/bin/g++ "-I/usr/local/include/opencv4/" "-fdiagnostics-color=always" "-g" "-std=c++2a" "main.cpp" "-o" "mainaaa" "-lfmt" "-lopencv_core" "-lopencv_imgcodecs" "-lopencv_highgui" "-lopencv_features2d" "-lopencv_imgproc"
#endif
