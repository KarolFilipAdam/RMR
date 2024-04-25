#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
///TOTO JE DEMO PROGRAM...AK SI HO NASIEL NA PC V LABAKU NEPREPISUJ NIC,ALE SKOPIRUJ SI MA NIEKAM DO INEHO FOLDERA
/// AK HO MAS Z GITU A ROBIS NA LABAKOVOM PC, TAK SI HO VLOZ DO FOLDERA KTORY JE JASNE ODLISITELNY OD TVOJICH KOLEGOV
/// NASLEDNE V POLOZKE Projects SKONTROLUJ CI JE VYPNUTY shadow build...
/// POTOM MIESTO TYCHTO PAR RIADKOV NAPIS SVOJE MENO ALEBO NEJAKY INY LUKRATIVNY IDENTIFIKATOR
/// KED SA NAJBLIZSIE PUSTIS DO PRACE, SKONTROLUJ CI JE MIESTO TOHTO TEXTU TVOJ IDENTIFIKATOR
/// AZ POTOM ZACNI ROBIT... AK TO NESPRAVIS, POJDU BODY DOLE... A NIE JEDEN,ALEBO DVA ALE BUDES RAD
/// AK SA DOSTANES NA SKUSKU


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress= "127.0.0.1";//192.168.1.11toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"
  //  cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter=0;
  //  timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    actIndex=-1;
    useCamera1=false;




    datacounter=0;


}

MainWindow::~MainWindow()
{
    delete ui;
}
bool deerFlag = false;
bool autoMove = false;
bool distanceFromGoal;
double error;
void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    ///prekreslujem obrazovku len vtedy, ked viem ze mam nove data. paintevent sa
    /// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::black);//cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero;
    pero.setStyle(Qt::SolidLine);//styl pera - plna ciara
    pero.setWidth(3);//hrubka pera -3pixely
    pero.setColor(Qt::green);//farba je zelena
    QRect rect;
    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit
    rect.translate(0,15);
    painter.drawRect(rect);

    if(useCamera1==true && actIndex>-1)/// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
    {
        std::cout<<actIndex<<std::endl;
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );//kopirovanie cvmat do qimage
        painter.drawImage(rect,image.rgbSwapped());
    }
    else
    {
        if(updateLaserPicture==1) ///ak mam nove data z lidaru
        {
            updateLaserPicture=0;

            painter.setPen(pero);
            //teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
         //   std::cout<<copyOfLaserData.numberOfScans<<std::endl;
            for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {
                //cout<<"dist"<<copyOfLaserData.Data[k].scanDistance<<endl;
                int dist=copyOfLaserData.Data[k].scanDistance/20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
                double uhol = 360.0-copyOfLaserData.Data[k].scanAngle;  // aky uhol zvierame
                int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x(); //prepocet do obrazovky
                int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();//prepocet do obrazovky
                if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                    painter.drawEllipse(QPoint(xp, yp),2,2);
            }
        }
    }

}


/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void  MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
     ui->lineEdit_2->setText(QString::number(robotX));
     ui->lineEdit_3->setText(QString::number(robotY));
     ui->lineEdit_4->setText(QString::number(robotFi));
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{
///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX
///

    ///tu mozete robit s datami z robota
    /// ale nic vypoctovo narocne - to iste vlakno ktore cita data z robota
    ///teraz tu posielam rychlosti na zaklade toho co setne joystick a vypisujeme data z robota(kazdy 5ty krat. ale mozete skusit aj castejsie). vyratajte si polohu. a vypiste spravnu
    /// tuto joystick cast mozete vklude vymazat,alebo znasilnit na vas regulator alebo ake mate pohnutky... kazdopadne, ak je pripojeny joystick tak aktualne to blokuje gombiky cize tak
    if(instance->count()>0)
    {
        if(forwardspeed==0 && rotationspeed!=0)
            robot.setRotationSpeed(rotationspeed);
        else if(forwardspeed!=0 && rotationspeed==0)
            robot.setTranslationSpeed(forwardspeed);
        else if((forwardspeed!=0 && rotationspeed!=0))
            robot.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
        else
            robot.setTranslationSpeed(0);

    }


    if(datacounter%5)
    {

        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
                // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
                //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
                //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
                /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
                /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
                /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        zadaniePrve(robotdata);
        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    datacounter++;

    return 0;

}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{


    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    updateLaserPicture=1;


    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

    Zadanie3();
    zadanieDruhe();
    //zad4();

    return 0;

}

///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z kamery
int MainWindow::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka
    updateLaserPicture=1;
    return 0;
}

///toto je calback na data zo skeleton trackera, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z trackera
int MainWindow::processThisSkeleton(skeleton skeledata)
{

    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

    updateSkeletonPicture=1;
    return 0;
}
void MainWindow::on_pushButton_9_clicked() //start button
{
    instance = QJoysticks::getInstance();

    forwardspeed=0;
    rotationspeed=0;
    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));

    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robot.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters(ipaddress,53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));
    //---simulator ma port 8889, realny robot 8000
    robot.setCameraParameters("http://"+ipaddress+":8000/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));
    robot.setSkeletonParameters("127.0.0.1",23432,23432,std::bind(&MainWindow::processThisSkeleton,this,std::placeholders::_1));
    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robot.robotStart();



    //ziskanie joystickov


    /// prepojenie joysticku s jeho callbackom... zas cez lambdu. neviem ci som to niekde spominal,ale lambdy su super. okrem toho mam este rad ternarne operatory a spolocneske hry ale to tiez nikoho nezaujima
    /// co vas vlastne zaujima? citanie komentov asi nie, inak by ste citali toto a ze tu je blbosti
    connect(
        instance, &QJoysticks::axisChanged,
        [this]( const int js, const int axis, const qreal value) { if(/*js==0 &&*/ axis==1){forwardspeed=-value*300;}
            if(/*js==0 &&*/ axis==0){rotationspeed=-value*(3.14159/2.0);}}
    );
}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    robot.setTranslationSpeed(500);

}

void MainWindow::on_pushButton_3_clicked() //back
{
    robot.setTranslationSpeed(-250);

}

void MainWindow::on_pushButton_6_clicked() //left
{
robot.setRotationSpeed(3.14159/2);

}

void MainWindow::on_pushButton_5_clicked()//right
{
robot.setRotationSpeed(-3.14159/2);

}

void MainWindow::on_pushButton_4_clicked() //stop
{
    robot.setTranslationSpeed(0);

}




void MainWindow::on_pushButton_clicked()
{
    if(useCamera1==true)
    {
        useCamera1=false;

        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera1=true;

        ui->pushButton->setText("use laser");
    }
}


double edgeAngle;
void MainWindow::getNewFrame()
{

}

double cordX = 0;
double cordY = 0;
double fi = 0;
double currentX = 0;
double currentY = 0;
long double tickToMeter = 0.000085292090497737556558; // [m/tick]
long double b = 0.23; // wheelbase distance in meters, from kobuki manual https://yujinrobot.github.io/kobuki/doxygen/enAppendixProtocolSpecification.html
double Kp = 2000;
double KpR = 3;

double natocenie = 0.3;
double ramVal = 10;
double rampa = ramVal;
double rotacia = 0;

bool edgeFlag = false;
bool wallFlag = false;
double bestDistance = 0;
double cord2X;
double cord2Y;
void MainWindow::zadaniePrve(TKobukiData robotdata){

    static unsigned short latestR = robotdata.EncoderRight;
    static unsigned short latestL = robotdata.EncoderLeft;
    short deltaR = robotdata.EncoderRight - latestR;
    short deltaL = robotdata.EncoderLeft - latestL;
    short l = ceil((double) (deltaL + deltaR) / 2);

    fi += (double) (deltaR - deltaL)  / b*tickToMeter;
    if (fi > PI){
        fi -= 2*PI;
    }
    if (fi < -PI){
        fi += 2*PI;
    }

    currentX += tickToMeter*l*cos(fi);
    currentY += tickToMeter*l*sin(fi);

    latestR = robotdata.EncoderRight;
    latestL = robotdata.EncoderLeft;

    double xDot = cordX - currentX;
    double yDot = cordY - currentY;

    if(autoMove){

        error = (abs(yDot) + abs(xDot));
        double theAngle = atan2(yDot,xDot);
        rotacia = theAngle - fi;

        if (rotacia > PI) rotacia = -2*PI+rotacia;
        if (rotacia < -PI) rotacia = 2*PI+rotacia;
        rotacia *= KpR;

        double zasah = Kp*error;
        double rychlost = zasah;


        if(rotacia > 1.5)
            rotacia = 1.5;
        if(rotacia < -1.5)
            rotacia = -1.5;
        if(rychlost > 600)
        {
            rychlost = 600;
        }
        if(rychlost-rampa > ramVal){
            rychlost = rampa+ramVal;
        }
        rampa = rychlost;


/////////////////////////////////////////////////// U1
        if(error > 0.03){
            if(abs(rotacia) > (30*PI/180)){    // musi sa otočiť
                robot.setRotationSpeed(rotacia);
                rampa = 10;
            }
            else{
                robot.setArcSpeed(rychlost,rychlost/rotacia); // ide rovno
            }
        }
        else{ //došiel
            robot.setTranslationSpeed(0);
            autoMove = false;
            if(edgeFlag){
                edgeFlag = false;
                deerFlag = false;
                cordX = cord2X;
                cordY = cord2Y;
            }
        }
//////////////////////////////////////////////////// U2

        /// Natoc sa na ciel
        /// Ak neni nič pred tebou hoď rovno
        /// ak je detekuj hranuy
        /// ak si detekoval hrany vypočitaj kratšiu, natoč sa a choď
        /// ak si endetekovaj hranu a je prkážka tak sa natoč na stenu a choď rovno po stene
        /// sleduj stenu do kým za error nezmensi potom opakuj ebod 1
        ///
        ///
        ///


////////////////////////////////////////////////////

    }
    else
        robot.setTranslationSpeed(0);

    emit uiValuesChanged(currentX, currentY, fi / PI * 180);


}

void MainWindow::on_pushButton_10_clicked()
{
    cordX = ui->lineEdit_5->text().toDouble();
    cordY = ui->lineEdit_6->text().toDouble();
    autoMove = true;
}

#define mapSize 60

void MainWindow::Zadanie3(){
    if(!ui->pushButton_11->isChecked())
        return;

    static bool map[mapSize*2][mapSize*2];
    static double x;
    static double y;
    static double latestAngle = fi;
    bool newData = false;

    if(abs(rotacia) > (10*PI/180)){    // musi sa otočiť
        return;
    }



    for (int i=0; i<copyOfLaserData.numberOfScans; i++) {

        if ((copyOfLaserData.Data[i].scanDistance < 160 ) || (copyOfLaserData.Data[i].scanDistance > 2000) ) continue;
        if ((copyOfLaserData.Data[i].scanDistance > 600 ) && (copyOfLaserData.Data[i].scanDistance < 800) ) continue;

        x = currentX + copyOfLaserData.Data[i].scanDistance*cos(fi+(double)(360-copyOfLaserData.Data[i].scanAngle)/180*PI)/1000;
        y = currentY + copyOfLaserData.Data[i].scanDistance*sin(fi+(double)(360-copyOfLaserData.Data[i].scanAngle)/180*PI)/1000;
        if (map[mapSize+(int)(x*mapSize/6)][mapSize+(int)(y*mapSize/6)] == 0) {
            map[mapSize+(int)(x*mapSize/6)][mapSize+(int)(y*mapSize/6)] = 1;
            newData = true;
        }
    }

    if (newData) {
        remove("file.txt");
        std::ofstream MyFile("file.txt");
        for (int stlpec=mapSize*2-1; stlpec>=0; stlpec--) {
            for (int riadok=0; riadok < mapSize*2-1; riadok++) {
                char character = map[riadok][stlpec] ? '#' : '0';
                MyFile << character;
                // std::cout << character;
            }
            MyFile << std::endl;
            //std::cout << std::endl;
        }
        MyFile.close();
    }


}


void MainWindow::on_pushButton_12_clicked()
{
    cordX = ui->lineEdit_7->text().toDouble();
    cordY = ui->lineEdit_8->text().toDouble();
    cord2X = cordX;
    cord2Y = cordY;
    autoMove = true;

}



std::pair<double,double> MainWindow::edgeFinder(){

    //if(((uhol < 45 ) || (uhol >315 )) && (dist > 5) && (dist>35))
    double previousX= 0;
    double previousY= 0;
    double previousDist = 9999999;
    bool skokX;
    bool skokY;
    double newEdgeY = 1;
    double newEdgeX = 1;
    double localBestX = 1000000;
    double localBestY = 1000000;
    double lowEdgeDist = 1;
    double localLowEdgeDist = 1000000;


    for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
    {
        int dist=copyOfLaserData.Data[k].scanDistance/20;
        double uhol = 360.0-copyOfLaserData.Data[k].scanAngle;

        if(((uhol < 45 ) || (uhol >315 )))
        {

            double x = currentX + dist*20*cos(fi+(double)(uhol)/180*PI)/1000;
            double y = currentY + dist*20*sin(fi+(double)(uhol)/180*PI)/1000;

            if(abs(dist - previousDist) > 10 && abs(dist-previousDist) < 9999)
            {
                newEdgeX = x;
                newEdgeY = y;
                lowEdgeDist = dist;

                if(lowEdgeDist < localLowEdgeDist)
                {
                    localLowEdgeDist = lowEdgeDist;
                    localBestX = newEdgeX;
                    localBestY = newEdgeY;
                }
            }
            previousDist = dist;

        }

    }
    pair<double,double> bestEdge;
    bestEdge.first = localBestX;
    bestEdge.second = localBestY;
    cout<<"localBestX "<<localBestX<<"localBestY "<<localBestY<<" "<<endl;

    return bestEdge;

}
double pastError;
double localError = 9999999;
void MainWindow::zadanieDruhe(){

    for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
    {
        int dist=copyOfLaserData.Data[k].scanDistance/20;
        double uhol = 360.0-copyOfLaserData.Data[k].scanAngle;



        if(!deerFlag){ // No deer

            if(((uhol < 45 ) || (uhol >315 )) && (dist > 5) && (dist<30)){  //prekazka predomnou
                cout<<"ZASTAV"<<endl;
                autoMove = false;
                distanceFromGoal = error;
                deerFlag = true;


            }
            else continue;

        }

        else
        { // determinoval som ze je predomnou prekazka

            if(wallFlag){

                wallFollower();   // ideme na stenu
            }
            else{

                pair<double,double> shortestDistances = edgeFinder(); // hladam najlblizsi edge

                if(shortestDistances.first == 1000000){ // nenasiel som idem na stenu
                    // deerFlag = false;
                    localError = 9999999;
                    wallFlag = true;
                    edgeFlag = false;
                }

                else{ // nasiel som idem k edgu

                    double x = shortestDistances.first;
                    double y = shortestDistances.second;

                    cout<<"x "<<x<<"y "<<y<<" "<<endl;

                    edgeFlag = true;
                    wallFlag = false;
                    cordX = x;
                    cordY = y;
                    autoMove = true;
                    deerFlag = false;  // uz nemam prekazku pred sebou lebo mam edge

                    // dojde na edge a znova determinuje



                }
            }




        }


    }


}

float ro = 0.5;


void MainWindow::wallFollower(){
    double Xzero = currentX - ro;
    double Yzero = currentY - ro;
    cout<<"stena"<<endl;
    if(localError < distanceFromGoal){
        wallFlag = false;
        deerFlag = false;
        cordX = cord2X;
        cordY = cord2Y;
        return;
    }
        if(abs(fi) > 45 && abs(fi) < 120){ //hor
            double Xzero = currentX + ro;
            double Yzero = currentY;
        }
        if( abs(fi) < 45 && abs(fi) > 120 ){ //ver val
            double Xzero = currentX;
            double Yzero = currentY + ro;
        }


        cordX = Xzero;
        cordY = Yzero;
        autoMove = true; // idem rovno

        // otoc 90
        //sleduj ci tam stale je a chod rovno
        // co ej to rovno ? ez iba translate speed
        //ak tam neni otoc o devedesiat najdi edge chod k edgu


        double dotX = cordX - currentX;
        double dotY = cordY - currentY;
        localError = (abs(dotY) + abs(dotX));





}
void MainWindow::on_pushButton_13_clicked()
{

}

