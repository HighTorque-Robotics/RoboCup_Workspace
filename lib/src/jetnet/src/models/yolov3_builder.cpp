#include "yolov3_builder.h"
#include "custom_assert.h"
#include "leaky_relu_plugin.h"
#include "leaky_relu_native.h"
#include "relu.h"
#include "logger.h"

using namespace jetnet;
using namespace nvinfer1;

template class Yolov3Builder<LeakyReluPlugin>;
template class Yolov3Builder<LeakyReluNative>;
template class Yolov3Builder<Relu>;

template<typename TActivation>
INetworkDefinition* Yolov3Builder<TActivation>::parse(DataType dt)
{
    m_logger->log(ILogger::Severity::kINFO, "Opening weights file '" + m_weightsfile + "'");
    m_weights = std::unique_ptr<DarknetWeightsLoader>(new DarknetWeightsLoader(dt));

    if (!m_weights->open(m_weightsfile)) {
        m_logger->log(ILogger::Severity::kERROR, "Failed Reading weights file '" + m_weightsfile + "'");
        return nullptr;
    }

    auto file_info = m_weights->get_file_info();
    m_logger->log(ILogger::Severity::kINFO, "Weights file info: V" + std::to_string(file_info.major) + "." +
                  std::to_string(file_info.minor) + "." + std::to_string(file_info.revision) + ", seen = " +
                  std::to_string(file_info.seen));

    // Note: assume the input blob is always provided with 32-bit floats
    ITensor* data = m_network->addInput(m_input_blob_name.c_str(), DataType::kFLOAT, m_input_dimensions);
    ASSERT(data);

    // Start of the network
    ILayer* conv0 = m_convs[0]("conv0", m_network, *m_weights, *data, 32, DimsHW{3, 3});
    ASSERT(conv0);  // 0

    // Downsample
    ILayer* conv1 = m_convs[1]("conv1", m_network, *m_weights, *conv0->getOutput(0), 64, DimsHW{3, 3}, DimsHW{1, 1}, DimsHW{2, 2});
    ASSERT(conv1);  // 1

    ILayer* conv2 = m_convs[2]("conv2", m_network, *m_weights, *conv1->getOutput(0), 32, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv2);  // 2

    ILayer* conv3 = m_convs[3]("conv3", m_network, *m_weights, *conv2->getOutput(0), 64, DimsHW{3, 3});
    ASSERT(conv3);  // 3

    ILayer* add0 = m_network->addElementWise(*conv3->getOutput(0), *conv1->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add0);   // 4
    add0->setName("add0");

    // Downsample
    ILayer* conv4 = m_convs[4]("conv4", m_network, *m_weights, *add0->getOutput(0), 128, DimsHW{3, 3}, DimsHW{1, 1}, DimsHW{2, 2});
    ASSERT(conv4);  // 5

    ILayer* conv5 = m_convs[5]("conv5", m_network, *m_weights, *conv4->getOutput(0), 64, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv5);  // 6

    ILayer* conv6 = m_convs[6]("conv6", m_network, *m_weights, *conv5->getOutput(0), 128, DimsHW{3, 3});
    ASSERT(conv6);  // 7

    ILayer* add1 = m_network->addElementWise(*conv6->getOutput(0), *conv4->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add1);   // 8
    add1->setName("add1");

    ILayer* conv7 = m_convs[7]("conv7", m_network, *m_weights, *add1->getOutput(0), 64, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv7);  // 9

    ILayer* conv8 = m_convs[8]("conv8", m_network, *m_weights, *conv7->getOutput(0), 128, DimsHW{3, 3});
    ASSERT(conv8);  // 10

    ILayer* add2 = m_network->addElementWise(*conv8->getOutput(0), *add1->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add2);   // 11
    add2->setName("add2");

    // Downsample
    ILayer* conv9 = m_convs[9]("conv9", m_network, *m_weights, *add2->getOutput(0), 256, DimsHW{3, 3}, DimsHW{1, 1}, DimsHW{2, 2});
    ASSERT(conv9);  // 12

    ILayer* conv10 = m_convs[10]("conv10", m_network, *m_weights, *conv9->getOutput(0), 128, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv10); // 13

    ILayer* conv11 = m_convs[11]("conv11", m_network, *m_weights, *conv10->getOutput(0), 256, DimsHW{3, 3});
    ASSERT(conv11); // 14

    ILayer* add3 = m_network->addElementWise(*conv11->getOutput(0), *conv9->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add3);   // 15
    add3->setName("add3");

    ILayer* conv12 = m_convs[12]("conv12", m_network, *m_weights, *add3->getOutput(0), 128, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv12); // 16

    ILayer* conv13 = m_convs[13]("conv13", m_network, *m_weights, *conv12->getOutput(0), 256, DimsHW{3, 3});
    ASSERT(conv13); // 17

    ILayer* add4 = m_network->addElementWise(*conv13->getOutput(0), *add3->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add4);   // 18
    add4->setName("add4");

    ILayer* conv14 = m_convs[14]("conv14", m_network, *m_weights, *add4->getOutput(0), 128, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv14); // 19

    ILayer* conv15 = m_convs[15]("conv15", m_network, *m_weights, *conv14->getOutput(0), 256, DimsHW{3, 3});
    ASSERT(conv15); // 20

    ILayer* add5 = m_network->addElementWise(*conv15->getOutput(0), *add4->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add5);   // 21
    add5->setName("add5");

    ILayer* conv16 = m_convs[16]("conv16", m_network, *m_weights, *add5->getOutput(0), 128, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv16); // 22

    ILayer* conv17 = m_convs[17]("conv17", m_network, *m_weights, *conv16->getOutput(0), 256, DimsHW{3, 3});
    ASSERT(conv17); // 23

    ILayer* add6 = m_network->addElementWise(*conv17->getOutput(0), *add5->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add6);   // 24
    add6->setName("add6");

    ILayer* conv18 = m_convs[18]("conv18", m_network, *m_weights, *add6->getOutput(0), 128, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv18); // 25

    ILayer* conv19 = m_convs[19]("conv19", m_network, *m_weights, *conv18->getOutput(0), 256, DimsHW{3, 3});
    ASSERT(conv19); // 26

    ILayer* add7 = m_network->addElementWise(*conv19->getOutput(0), *add6->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add7);   // 27
    add7->setName("add7");

    ILayer* conv20 = m_convs[20]("conv20", m_network, *m_weights, *add7->getOutput(0), 128, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv20); // 28

    ILayer* conv21 = m_convs[21]("conv21", m_network, *m_weights, *conv20->getOutput(0), 256, DimsHW{3, 3});
    ASSERT(conv21); // 29

    ILayer* add8 = m_network->addElementWise(*conv21->getOutput(0), *add7->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add8);   // 30
    add8->setName("add8");

    ILayer* conv22 = m_convs[22]("conv22", m_network, *m_weights, *add8->getOutput(0), 128, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv22); // 31

    ILayer* conv23 = m_convs[23]("conv23", m_network, *m_weights, *conv22->getOutput(0), 256, DimsHW{3, 3});
    ASSERT(conv23); // 32

    ILayer* add9 = m_network->addElementWise(*conv23->getOutput(0), *add8->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add9);   // 33
    add9->setName("add9");

    ILayer* conv24 = m_convs[24]("conv24", m_network, *m_weights, *add9->getOutput(0), 128, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv24); // 34

    ILayer* conv25 = m_convs[25]("conv25", m_network, *m_weights, *conv24->getOutput(0), 256, DimsHW{3, 3});
    ASSERT(conv25); // 35

    ILayer* add10 = m_network->addElementWise(*conv25->getOutput(0), *add9->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add10);  // 36
    add10->setName("add10");

    // feed forward to concat1 ***
    // Downsample
    ILayer* conv26 = m_convs[26]("conv26", m_network, *m_weights, *add10->getOutput(0), 512, DimsHW{3, 3}, DimsHW{1, 1}, DimsHW{2, 2});
    ASSERT(conv26); // 37

    ILayer* conv27 = m_convs[27]("conv27", m_network, *m_weights, *conv26->getOutput(0), 256, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv27); // 38

    ILayer* conv28 = m_convs[28]("conv28", m_network, *m_weights, *conv27->getOutput(0), 512, DimsHW{3, 3});
    ASSERT(conv28); // 39

    ILayer* add11 = m_network->addElementWise(*conv28->getOutput(0), *conv26->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add11);  // 40
    add11->setName("add11");

    ILayer* conv29 = m_convs[29]("conv29", m_network, *m_weights, *add11->getOutput(0), 256, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv29); // 41

    ILayer* conv30 = m_convs[30]("conv30", m_network, *m_weights, *conv29->getOutput(0), 512, DimsHW{3, 3});
    ASSERT(conv30); // 42

    ILayer* add12 = m_network->addElementWise(*conv30->getOutput(0), *add11->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add12);  // 43
    add12->setName("add12");

    ILayer* conv31 = m_convs[31]("conv31", m_network, *m_weights, *add12->getOutput(0), 256, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv31); // 44

    ILayer* conv32 = m_convs[32]("conv32", m_network, *m_weights, *conv31->getOutput(0), 512, DimsHW{3, 3});
    ASSERT(conv32); // 45

    ILayer* add13 = m_network->addElementWise(*conv32->getOutput(0), *add12->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add13);  // 46
    add13->setName("add13");

    ILayer* conv33 = m_convs[33]("conv33", m_network, *m_weights, *add13->getOutput(0), 256, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv33); // 47

    ILayer* conv34 = m_convs[34]("conv34", m_network, *m_weights, *conv33->getOutput(0), 512, DimsHW{3, 3});
    ASSERT(conv34); // 48

    ILayer* add14 = m_network->addElementWise(*conv34->getOutput(0), *add13->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add14);  // 49
    add14->setName("add14");

    ILayer* conv35 = m_convs[35]("conv35", m_network, *m_weights, *add14->getOutput(0), 256, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv35); // 50

    ILayer* conv36 = m_convs[36]("conv36", m_network, *m_weights, *conv35->getOutput(0), 512, DimsHW{3, 3});
    ASSERT(conv36); // 51

    ILayer* add15 = m_network->addElementWise(*conv36->getOutput(0), *add14->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add15);  // 52
    add15->setName("add15");

    ILayer* conv37 = m_convs[37]("conv37", m_network, *m_weights, *add15->getOutput(0), 256, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv37); // 53

    ILayer* conv38 = m_convs[38]("conv38", m_network, *m_weights, *conv37->getOutput(0), 512, DimsHW{3, 3});
    ASSERT(conv38); // 54

    ILayer* add16 = m_network->addElementWise(*conv38->getOutput(0), *add15->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add16);  // 55
    add16->setName("add16");

    ILayer* conv39 = m_convs[39]("conv39", m_network, *m_weights, *add16->getOutput(0), 256, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv39); // 56

    ILayer* conv40 = m_convs[40]("conv40", m_network, *m_weights, *conv39->getOutput(0), 512, DimsHW{3, 3});
    ASSERT(conv40); // 57

    ILayer* add17 = m_network->addElementWise(*conv40->getOutput(0), *add16->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add17);  // 58
    add17->setName("add17");

    ILayer* conv41 = m_convs[41]("conv41", m_network, *m_weights, *add17->getOutput(0), 256, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv41); // 59

    ILayer* conv42 = m_convs[42]("conv42", m_network, *m_weights, *conv41->getOutput(0), 512, DimsHW{3, 3});
    ASSERT(conv42); // 60

    ILayer* add18 = m_network->addElementWise(*conv42->getOutput(0), *add17->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add18);  // 61
    add18->setName("add18");

    // feed forward to concat0 ***
    // Downsample
    ILayer* conv43 = m_convs[43]("conv43", m_network, *m_weights, *add18->getOutput(0), 1024, DimsHW{3, 3}, DimsHW{1, 1}, DimsHW{2, 2});
    ASSERT(conv43); // 62

    ILayer* conv44 = m_convs[44]("conv44", m_network, *m_weights, *conv43->getOutput(0), 512, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv44); // 63

    ILayer* conv45 = m_convs[45]("conv45", m_network, *m_weights, *conv44->getOutput(0), 1024, DimsHW{3, 3});
    ASSERT(conv45); // 64

    ILayer* add19 = m_network->addElementWise(*conv45->getOutput(0), *conv43->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add19);  // 65
    add19->setName("add19");

    ILayer* conv46 = m_convs[46]("conv46", m_network, *m_weights, *add19->getOutput(0), 512, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv46); // 66

    ILayer* conv47 = m_convs[47]("conv47", m_network, *m_weights, *conv46->getOutput(0), 1024, DimsHW{3, 3});
    ASSERT(conv47); // 67

    ILayer* add20 = m_network->addElementWise(*conv47->getOutput(0), *add19->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add20);  // 68
    add20->setName("add20");

    ILayer* conv48 = m_convs[48]("conv48", m_network, *m_weights, *add20->getOutput(0), 512, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv48); // 69

    ILayer* conv49 = m_convs[49]("conv49", m_network, *m_weights, *conv48->getOutput(0), 1024, DimsHW{3, 3});
    ASSERT(conv49); // 70

    ILayer* add21 = m_network->addElementWise(*conv49->getOutput(0), *add20->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add21);  // 71
    add21->setName("add21");

    ILayer* conv50 = m_convs[50]("conv50", m_network, *m_weights, *add21->getOutput(0), 512, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv50); // 72

    ILayer* conv51 = m_convs[51]("conv51", m_network, *m_weights, *conv50->getOutput(0), 1024, DimsHW{3, 3});
    ASSERT(conv51); // 73

    ILayer* add22 = m_network->addElementWise(*conv51->getOutput(0), *add21->getOutput(0), ElementWiseOperation::kSUM);
    ASSERT(add22);  // 74
    add22->setName("add22");


    ILayer* conv52 = m_convs[52]("conv52", m_network, *m_weights, *add22->getOutput(0), 512, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv52); // 75

    ILayer* conv53 = m_convs[53]("conv53", m_network, *m_weights, *conv52->getOutput(0), 1024, DimsHW{3, 3});
    ASSERT(conv53); // 76

    ILayer* conv54 = m_convs[54]("conv54", m_network, *m_weights, *conv53->getOutput(0), 512, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv54); // 77

    ILayer* conv55 = m_convs[55]("conv55", m_network, *m_weights, *conv54->getOutput(0), 1024, DimsHW{3, 3});
    ASSERT(conv55); // 78

    ILayer* conv56 = m_convs[56]("conv56", m_network, *m_weights, *conv55->getOutput(0), 512, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv56); // 79

    const int conv57_num_filters = 1024;
    ILayer* conv57 = m_convs[57]("conv57", m_network, *m_weights, *conv56->getOutput(0), conv57_num_filters, DimsHW{3, 3});
    ASSERT(conv57); // 80

    // last conv layers are convolution only (no batch norm, no activation)
    DimsHW conv72_kernel_size{1, 1};
    const int conv72_num_filters = m_output_large.num_anchors * (1 + m_output_large.num_coords + m_output_large.num_classes);
    Weights conv72_biases = m_weights->get(conv72_num_filters);
    Weights conv72_weights = m_weights->get(conv72_num_filters * conv57_num_filters * conv72_kernel_size.h() * conv72_kernel_size.w());
    ILayer* conv72 = m_network->addConvolution(*conv57->getOutput(0), conv72_num_filters, conv72_kernel_size, conv72_weights,
                                               conv72_biases);
    ASSERT(conv72); // 81
    conv72->setName("conv72");

    // we implement the 'yolo' layer as just a sigmoid activation layer since x,y objectness and class scores need sigmoid activation.
    // width/height are calculated using the exponent of the w/h feature maps (second and third channel of the output tensor) i.s.o.
    // applying the sigmoid operator.
    // since we can prove that exp(x) = s(x) / (1 - s(x)) where s is the sigmoid operator
    // we can safely apply sigmoid on the w/h maps too and still deduct the width/height coordinates
    // in that way we can activate the whole output tensor which simplifies the implementation of the 'yolo' layer
    ILayer* yolo0 = m_network->addActivation(*conv72->getOutput(0), ActivationType::kSIGMOID);
    ASSERT(yolo0);  // 82
    yolo0->setName("yolo0");

    // first endpoint for large objects
    yolo0->getOutput(0)->setName(m_output_large.blob_name.c_str());
    m_network->markOutput(*yolo0->getOutput(0));


    ILayer* conv58 = m_convs[58]("conv58", m_network, *m_weights, *conv56->getOutput(0), 256, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv58); // 84

    m_upsample_plugin0 = std::unique_ptr<UpsamplePlugin>(new UpsamplePlugin(2));
    ASSERT(m_upsample_plugin0);

    ITensor* conv58_tensor = conv58->getOutput(0);
    ILayer* upsample0 = m_network->addPlugin(&conv58_tensor, 1, *m_upsample_plugin0);
    ASSERT(upsample0);  // 85
    upsample0->setName("upsample0");

    // Feed forward from add18
    ITensor* concat_tensors0[] = {upsample0->getOutput(0), add18->getOutput(0)};
    ILayer* concat0 = m_network->addConcatenation(concat_tensors0, 2);
    ASSERT(concat0);    // 86
    concat0->setName("concat0");


    ILayer* conv59 = m_convs[59]("conv59", m_network, *m_weights, *concat0->getOutput(0), 256, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv59); // 87

    ILayer* conv60 = m_convs[60]("conv60", m_network, *m_weights, *conv59->getOutput(0), 512, DimsHW{3, 3});
    ASSERT(conv60); // 88

    ILayer* conv61 = m_convs[61]("conv61", m_network, *m_weights, *conv60->getOutput(0), 256, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv61); // 89

    ILayer* conv62 = m_convs[62]("conv62", m_network, *m_weights, *conv61->getOutput(0), 512, DimsHW{3, 3});
    ASSERT(conv62); // 90

    ILayer* conv63 = m_convs[63]("conv63", m_network, *m_weights, *conv62->getOutput(0), 256, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv63); // 91

    const int conv64_num_filters = 512;
    ILayer* conv64 = m_convs[64]("conv64", m_network, *m_weights, *conv63->getOutput(0), conv64_num_filters , DimsHW{3, 3});
    ASSERT(conv64); // 92

    // last conv layers are convolution only (no batch norm, no activation)
    DimsHW conv73_kernel_size{1, 1};
    const int conv73_num_filters = m_output_mid.num_anchors * (1 + m_output_mid.num_coords + m_output_mid.num_classes);
    Weights conv73_biases = m_weights->get(conv73_num_filters);
    Weights conv73_weights = m_weights->get(conv73_num_filters * conv64_num_filters * conv73_kernel_size.h() * conv73_kernel_size.w());
    ILayer* conv73 = m_network->addConvolution(*conv64->getOutput(0), conv73_num_filters, conv73_kernel_size, conv73_weights,
                                               conv73_biases);
    ASSERT(conv73); // 93
    conv73->setName("conv73");

    // second 'yolo' layer
    ILayer* yolo1 = m_network->addActivation(*conv73->getOutput(0), ActivationType::kSIGMOID);
    ASSERT(yolo1);  // 94
    yolo1->setName("yolo1");

    // second endpoint for medium sized objects
    yolo1->getOutput(0)->setName(m_output_mid.blob_name.c_str());
    m_network->markOutput(*yolo1->getOutput(0));


    ILayer* conv65 = m_convs[65]("conv65", m_network, *m_weights, *conv63->getOutput(0), 128, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv65);

    m_upsample_plugin1 = std::unique_ptr<UpsamplePlugin>(new UpsamplePlugin(2));
    ASSERT(m_upsample_plugin1); // 96

    ITensor* conv65_tensor = conv65->getOutput(0);
    ILayer* upsample1 = m_network->addPlugin(&conv65_tensor, 1, *m_upsample_plugin1);
    ASSERT(upsample1);  // 97
    upsample1->setName("upsample1");

    // Feed forward from add10
    ITensor* concat_tensors1[] = {upsample1->getOutput(0), add10->getOutput(0)};
    ILayer* concat1 = m_network->addConcatenation(concat_tensors1, 2);
    ASSERT(concat1);    // 98
    concat1->setName("concat1");


    ILayer* conv66 = m_convs[66]("conv66", m_network, *m_weights, *concat1->getOutput(0), 128, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv66); // 99

    ILayer* conv67 = m_convs[67]("conv67", m_network, *m_weights, *conv66->getOutput(0), 256, DimsHW{3, 3});
    ASSERT(conv67); // 100

    ILayer* conv68 = m_convs[68]("conv68", m_network, *m_weights, *conv67->getOutput(0), 128, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv68); // 101

    ILayer* conv69 = m_convs[69]("conv69", m_network, *m_weights, *conv68->getOutput(0), 256, DimsHW{3, 3});
    ASSERT(conv69); // 102

    ILayer* conv70 = m_convs[70]("conv70", m_network, *m_weights, *conv69->getOutput(0), 128, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv70); // 103

    const int conv71_num_filters = 256;
    ILayer* conv71 = m_convs[71]("conv71", m_network, *m_weights, *conv70->getOutput(0), conv71_num_filters, DimsHW{3, 3});
    ASSERT(conv71); // 104

    // last conv layers are convolution only (no batch norm, no activation)
    DimsHW conv74_kernel_size{1, 1};
    const int conv74_num_filters = m_output_small.num_anchors * (1 + m_output_small.num_coords + m_output_small.num_classes);
    Weights conv74_biases = m_weights->get(conv74_num_filters);
    Weights conv74_weights = m_weights->get(conv74_num_filters * conv71_num_filters * conv74_kernel_size.h() * conv74_kernel_size.w());
    ILayer* conv74 = m_network->addConvolution(*conv71->getOutput(0), conv74_num_filters, conv74_kernel_size, conv74_weights,
                                               conv74_biases);
    ASSERT(conv74); // 105
    conv74->setName("conv74");

    // third 'yolo' layer
    ILayer* yolo2 = m_network->addActivation(*conv74->getOutput(0), ActivationType::kSIGMOID);
    ASSERT(yolo2);  // 106
    yolo2->setName("yolo2");

    // third endpoint for small sized objects
    yolo2->getOutput(0)->setName(m_output_small.blob_name.c_str());
    m_network->markOutput(*yolo2->getOutput(0));

    return m_network;
}
