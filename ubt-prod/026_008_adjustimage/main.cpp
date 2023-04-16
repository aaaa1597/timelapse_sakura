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
    fs::remove_all("/media/sf_timelapse_sakura/pictures/026_009_makermove");
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

        /* 拡縮率を求める */
        double baselent2b = std::sqrt( (double)(basetop.x -basebtm.x )*(basetop.x -basebtm.x)  + (basetop.y -basebtm.y) *(basetop.y -basebtm.y ) );
        double makerlent2b= std::sqrt( (double)(makertop.x-makerbtm.x)*(makertop.x-makerbtm.x) + (makertop.y-makerbtm.y)*(makertop.y-makerbtm.y) );
        double scale = (double)makerlent2b / baselent2b;
        /* マーカーの長さ(makerlen-t2r, makerlen-b2r)を求める */
        double makerlent2r = baselent2r * scale;
        double makerlenb2r = baselenb2r * scale;
        std::cout << fmt::format("111 拡縮率を求める scale(\t{}\t) t2b(base:\t{}\t, maker:\t{}\t) makerlen-t2r(\t{}\t) makerlen-b2r(\t{}\t)", scale, baselent2b,makerlent2b, makerlent2r, makerlenb2r) << std::endl;

        /* 回転/拡縮の中心点を求める */
        // https://memo.sugyan.com/entry/20090408/1239148436
        // https://qiita.com/UchiwaFuujinn/items/12cb4de120ac5af292f8
        // https://okwave.jp/qa/q8906319.html
        /*                                                 */
        /*          maker-top                              */
        /*                ↑                               */
        /*              △｜                               */
        /*             /                                   */
        /*            /                                    */
        /*           /                                     */
        /*          /                                      */
        /*     ↑  /                                       */
        /*     ｜ /                                        */
        /*       △ maker-btm                              */
        /*       ↑この点を点oに移動 : 移動量(offsetx,offsety) */
        /*                                          */
        /* 準備1.基準線を点Oに移動(移動量は保持っとく) */
        int offsetx = makerbtm.x;
        int offsety = makerbtm.y;
        cv::Point2f makertoptrans = cv::Point2f(makertop.x-offsetx, makertop.y-offsety);/* pointbtm分のOffset */
        cv::Point2f makerbtmtrans = cv::Point2f(makerbtm.x-offsetx, makerbtm.y-offsety);/* pointbtmを点Oにする */
        std::cout << fmt::format("222 tmp移動量求め移動 offset(\t{}\t, \t{}\t) \nmaker-top-trans(\t{}\t, \t{}\t) maker-btm-trans(\t{}\t, \t{}\t)", offsetx,offsety, makertoptrans.x,makertoptrans.y, makerbtmtrans.x,makerbtmtrans.y) << std::endl;

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
        double tilt = (double)makertoptrans.y / makertoptrans.x;
        double theta = (makertoptrans.x < 0) ? M_PI + std::atan(tilt) : std::atan(tilt);
        std::cout << fmt::format("333 tmp回転量を求める theta(\t{}\t)", theta) << std::endl;
        /* x′=xcos(θ)−ysin(θ) */
        /* y′=xsin(θ)+ycos(θ) */
        double makertoptransrotx = makertoptrans.x * std::cos(-theta) - makertoptrans.y * std::sin(-theta);
        double makertoptransroty = makertoptrans.x * std::sin(-theta) + makertoptrans.y * std::cos(-theta);
        cv::Point2f makertoptransrot = cv::Point2f(makertoptransrotx, makertoptransroty);   /* x軸上に回転 */
        cv::Point2f makerbtmtransrot = makerbtmtrans;                                       /* 点Oのまま */
        std::cout << fmt::format("444 tmp回転 maker-top-transrot(\t{}\t, \t{}\t) maker-btm-transrot(\t{}\t, \t{}\t)", makertoptransrot.x,makertoptransrot.y, makerbtmtransrot.x,makerbtmtransrot.y) << std::endl;

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
        /* centerpostransrotx = (-t2r^2 + b2r^2 + top.x^2) / (2 * top.x) */
        /* centerpostransroty = sqrt(b2r^2 - centerpostransrotx^2) = sqrt(t2r^2 - makertoptransrot.x^2 + 2*makertoptransrot.x*centerpostransrotx - centerpostransrotx^2) */
        double centerpostransrotx = (-(makerlent2r*makerlent2r) + (makerlenb2r*makerlenb2r) + (makertoptransrot.x*makertoptransrot.x)) / (2 * makertoptransrot.x);
        double centerpostransroty = std::sqrt((makerlenb2r*makerlenb2r) - (centerpostransrotx*centerpostransrotx));
        double centerpostransroty2= std::sqrt((makerlent2r*makerlent2r) - (makertoptransrot.x*makertoptransrot.x) + 2*makertoptransrot.x*centerpostransrotx - (centerpostransrotx*centerpostransrotx));
        std::cout << fmt::format("555 (仮)中心座標算出 tmp(\t{}\t, \t{}\t, \t{}\t)", centerpostransrotx, centerpostransroty, centerpostransroty2) << std::endl;

        /* 準備4.回転を元に戻す */
        double centerpostransx = centerpostransrotx * std::cos(theta) - centerpostransroty * std::sin(theta);
        double centerpostransy = centerpostransrotx * std::sin(theta) + centerpostransroty * std::cos(theta);
        cv::Point2f centerpostrans = cv::Point2f(centerpostransx, centerpostransy);

        double makertoptransx_r = makertoptransrot.x * std::cos(theta) - makertoptransrot.y * std::sin(theta);
        double makertoptransy_r = makertoptransrot.x * std::sin(theta) + makertoptransrot.y * std::cos(theta);
        cv::Point2f makertoptrans_r = cv::Point2f(makertoptransx_r, makertoptransy_r);
        cv::Point2f makerbtmtrans_r = makerbtmtransrot;                               /* 点Oのまま */
        std::cout << fmt::format("666 tmp回転戻し makertop(\t{}\t, \t{}\t) makerbtm(\t{}\t, \t{}\t) centerpos(\t{}\t, \t{}\t)", makertoptrans_r.x,makertoptrans_r.y, makerbtmtrans_r.x,makerbtmtrans_r.y, centerpostrans.x,centerpostrans.y) << std::endl;

        /* 準備5.移動を元に戻す */
        cv::Point2f centerposprev = cv::Point2f(centerpostrans.x+offsetx, centerpostrans.y+offsety);
        cv::Point2f makertop_r = cv::Point2f(makertoptrans_r.x+offsetx, makertoptrans_r.y+offsety);
        cv::Point2f makerbtm_r = cv::Point2f(makerbtmtrans_r.x+offsetx, makerbtmtrans_r.y+offsety);
        std::cout << fmt::format("777 tmp移動戻し makertop(\t{}\t, \t{}\t) makerbtm(\t{}\t, \t{}\t) centerposprev(\t{}\t, \t{}\t)", makertop_r.x,makertop_r.y, makerbtm_r.x,makerbtm_r.y, centerposprev.x,centerposprev.y) << std::endl;

        /* 拡縮量を求める */
//      double scale = (double)makerlent2b / baselent2b;    /* すでに算出済み */

        /* 回転量を求める(y軸とマーカー線から) */
        double rotatetilt = ((double)makertop.y-makerbtm.y) / (makertop.x-makerbtm.x);
        double rotaterad = std::atan(rotatetilt);
        double rotatedegree = (rotaterad < 0) ? (90) + rotaterad * (180/M_PI) : (90) - rotaterad * (180/M_PI);
        std::cout << fmt::format("888 回転量求める rotatedegree(\t{}\t) rotaterad(\t{}\t) rotatetilt(\t{}\t) makertop(\t{}\t, \t{}\t) makerbtm(\t{}\t, \t{}\t)", rotatedegree, rotaterad, rotatetilt, makertop.x,makertop.y, makerbtm.x,makerbtm.y) << std::endl;

        /* 移動量を求める(回転/拡縮の中心を、ベース画像とマーカ画像で合わせる) */
        double movex = centerposprev.x - baseright.x;
        double movey = centerposprev.y - baseright.y;
        cv::Point2f centerposcomplete  = cv::Point2f(centerposprev.x-movex, centerposprev.y-movey);
        cv::Point2f makertop_rcomplete = cv::Point2f(makertop_r.x   -movex, makertop_r.y   -movey);
        cv::Point2f makerbtm_rcomplete = cv::Point2f(makerbtm_r.x   -movex, makerbtm_r.y   -movey);
        std::cout << fmt::format("999 移動量を求める move(\t{}\t, \t{}\t) centerposcomplete(\t{}\t, \t{}\t) makertop_rcomplete(\t{}\t, \t{}\t) makerbtm_rcomplete(\t{}\t, \t{}\t)", movex,movey, centerposcomplete.x,centerposcomplete.y, makertop_rcomplete.x,makertop_rcomplete.y, makerbtm_r.x,makerbtm_r.y) << std::endl;
        std::cout << fmt::format("aaa 材料揃った move(\t{}\t, \t{}\t) centerposcomplete(\t{}\t, \t{}\t) rotatedegree(\t{}\t) scale(\t{}\t)", movex, movey, centerposcomplete.x,centerposcomplete.y, rotatedegree, scale) << std::endl;

        /* マーカー画像読込み 読込み済 */
//      cv::Mat4b makerimg = cv::imread(makerfilefullpath, cv::IMREAD_UNCHANGED);
//      /* Target画像読込み(Open) */
//      cv::Mat4b targetimg = cv::imread(makerfilefullpath, cv::IMREAD_UNCHANGED);
//      if(targetimg.empty()) {
//          std::cout << fmt::format("Target画像の読み込みに失敗!! {}", makerfilefullpath) << std::endl;
//          return 1;
//      }
//      std::cout << fmt::format("Target画像 サイズ={}x{} channels: {}", targetimg.cols, targetimg.rows, targetimg.channels()) << std::endl;

        /* 移動行列生成 */
        cv::Mat moveMatrix = (cv::Mat_<double>(2,3) << 1.0, 0.0, -movex, 0.0, 1.0, -movey);

        /* 平行移動を実行 */
        cv::Mat4b makermoveimg;
        cv::warpAffine(makerimg, makermoveimg, moveMatrix, makerimg.size(), cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
//      cv::Mat4b targetmoveimg;
//      cv::warpAffine(targetimg, targetmoveimg, moveMatrix, targetimg.size(), cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        /* マーカー画像の平行移動を出力先に保存 */
        std::string makermovepath = file.path().parent_path().parent_path().string() + "/026_009_makermove/";
        fs::create_directories(makermovepath);
        std::string makermovefilefullpath = file.path().parent_path().parent_path().string() + "/026_009_makermove/" + file.path().stem().string() + ".png";
        cv::imwrite(makermovefilefullpath, makermoveimg);

        {
        cv::Mat image = cv::Mat::zeros(cv::Size(2560,1280), CV_8UC3);
        cv::line(image, centerposcomplete, makertop_rcomplete, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        cv::line(image, centerposcomplete, makerbtm_rcomplete, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
        std::string makercompletepath = file.path().parent_path().parent_path().string() + "/026_011_makercomplete/";
        fs::create_directories(makercompletepath);
        std::string makercompletefilefullpath = file.path().parent_path().parent_path().string() + "/026_011_makercomplete/" + file.path().stem().string() + ".png";
        cv::imwrite(makercompletefilefullpath, image);
        }

        /* 回転&拡縮の行列取得 */
        cv::Mat rotscaleMatrix = cv::getRotationMatrix2D(centerposcomplete, -rotatedegree, 1/scale);
        /* 回転&拡縮 実行 */
        cv::Mat4b makermoverotscaleimg;
        cv::warpAffine(makermoveimg, makermoverotscaleimg, rotscaleMatrix, makermoveimg.size(), cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
//      cv::Mat4b targetmoverotscaleimg;
//      cv::warpAffine(targetmoveimg, targetmoverotscaleimg, rotscaleMatrix, targetmoveimg.size(), cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        /* 加工完了マーカー画像を出力先に保存 */
        std::string makercompletepath = file.path().parent_path().parent_path().string() + "/026_010_makercomplete/";
        fs::create_directories(makercompletepath);
        std::string makercompletefilefullpath = file.path().parent_path().parent_path().string() + "/026_010_makercomplete/" + file.path().stem().string() + ".png";
        cv::imwrite(makercompletefilefullpath, makermoverotscaleimg);

        // if(file.path().stem().string() == "IMG_20230325_135438_1007") {
        //     double rotdelta   = (rotatedegree/10);
        //     double scaledelta = 1.0;
        //     for(int lpctaaa = 0; lpctaaa < 10; lpctaaa++, rotdelta+=(rotatedegree/10), scaledelta-=(scale/10)) {
        //         /* 回転&拡縮の行列取得 */
        //         cv::Mat rotscaleMatrix = cv::getRotationMatrix2D(centerposcomplete, -rotdelta, 1/scaledelta);
        //         /* 回転&拡縮 実行 */
        //         cv::Mat4b aaaaa;
        //         cv::warpAffine(makermoveimg, aaaaa, rotscaleMatrix, makermoveimg.size(), cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        //         /* 加工完了マーカー画像を出力先に保存 */
        //         std::string aaadir = file.path().parent_path().parent_path().string() + "/026_010_makercomplete/";
        //         fs::create_directories(aaadir);
        //         std::string aaafullpath = file.path().parent_path().parent_path().string() + "/026_010_makercomplete/" + file.path().stem().string() + fmt::format("___{}", lpctaaa) + ".png";
        //         cv::imwrite(aaafullpath, aaaaa);
        //     }
        //     int aaa = 0;
        // }
    }

    return 0;
}

#if 0
cd ubt-prod/026_008_adjustimage
/usr/bin/g++ "-I/usr/local/include/opencv4/" "-fdiagnostics-color=always" "-g" "-std=c++2a" "main.cpp" "-o" "mainaaa" "-lfmt" "-lopencv_core" "-lopencv_imgcodecs" "-lopencv_highgui" "-lopencv_features2d" "-lopencv_imgproc"
#endif
