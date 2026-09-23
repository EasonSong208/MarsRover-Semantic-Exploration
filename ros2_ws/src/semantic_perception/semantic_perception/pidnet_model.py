"""PIDNet-S inference model used by the hazard5 semantic node.

The module is a deployment-only reduction of the official PIDNet implementation
by Jiacong Xu.  It preserves the module names and tensor shapes used by the V3
training checkpoint while omitting training-only auxiliary heads and other model
sizes.
"""

import torch
from torch import nn
import torch.nn.functional as functional


BN_MOMENTUM = 0.1


class BasicBlock(nn.Module):
    """PIDNet residual basic block."""

    expansion = 1

    def __init__(
        self,
        inplanes,
        planes,
        stride=1,
        downsample=None,
        no_relu=False,
    ):
        super().__init__()
        self.conv1 = nn.Conv2d(
            inplanes, planes, kernel_size=3, stride=stride,
            padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(planes, momentum=BN_MOMENTUM)
        self.relu = nn.ReLU(inplace=True)
        self.conv2 = nn.Conv2d(
            planes, planes, kernel_size=3, padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(planes, momentum=BN_MOMENTUM)
        self.downsample = downsample
        self.stride = stride
        self.no_relu = no_relu

    def forward(self, inputs):
        """Apply one residual block."""
        residual = inputs
        outputs = self.relu(self.bn1(self.conv1(inputs)))
        outputs = self.bn2(self.conv2(outputs))
        if self.downsample is not None:
            residual = self.downsample(inputs)
        outputs += residual
        return outputs if self.no_relu else self.relu(outputs)


class Bottleneck(nn.Module):
    """PIDNet residual bottleneck block."""

    expansion = 2

    def __init__(
        self,
        inplanes,
        planes,
        stride=1,
        downsample=None,
        no_relu=True,
    ):
        super().__init__()
        self.conv1 = nn.Conv2d(inplanes, planes, kernel_size=1, bias=False)
        self.bn1 = nn.BatchNorm2d(planes, momentum=BN_MOMENTUM)
        self.conv2 = nn.Conv2d(
            planes, planes, kernel_size=3, stride=stride,
            padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(planes, momentum=BN_MOMENTUM)
        self.conv3 = nn.Conv2d(
            planes, planes * self.expansion, kernel_size=1, bias=False)
        self.bn3 = nn.BatchNorm2d(
            planes * self.expansion, momentum=BN_MOMENTUM)
        self.relu = nn.ReLU(inplace=True)
        self.downsample = downsample
        self.stride = stride
        self.no_relu = no_relu

    def forward(self, inputs):
        """Apply one bottleneck block."""
        residual = inputs
        outputs = self.relu(self.bn1(self.conv1(inputs)))
        outputs = self.relu(self.bn2(self.conv2(outputs)))
        outputs = self.bn3(self.conv3(outputs))
        if self.downsample is not None:
            residual = self.downsample(inputs)
        outputs += residual
        return outputs if self.no_relu else self.relu(outputs)


class SegmentHead(nn.Module):
    """PIDNet semantic output head."""

    def __init__(self, inplanes, interplanes, outplanes):
        super().__init__()
        self.bn1 = nn.BatchNorm2d(inplanes, momentum=BN_MOMENTUM)
        self.conv1 = nn.Conv2d(
            inplanes, interplanes, kernel_size=3, padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(interplanes, momentum=BN_MOMENTUM)
        self.relu = nn.ReLU(inplace=True)
        self.conv2 = nn.Conv2d(interplanes, outplanes, kernel_size=1, bias=True)
        self.scale_factor = None

    def forward(self, inputs):
        """Produce semantic logits."""
        outputs = self.conv1(self.relu(self.bn1(inputs)))
        return self.conv2(self.relu(self.bn2(outputs)))


class PAPPM(nn.Module):
    """Parallel aggregation pyramid pooling module used by PIDNet-S."""

    def __init__(self, inplanes, branch_planes, outplanes):
        super().__init__()
        self.scale1 = self._pooled_scale(inplanes, branch_planes, 5, 2, 2)
        self.scale2 = self._pooled_scale(inplanes, branch_planes, 9, 4, 4)
        self.scale3 = self._pooled_scale(inplanes, branch_planes, 17, 8, 8)
        self.scale4 = nn.Sequential(
            nn.AdaptiveAvgPool2d((1, 1)),
            nn.BatchNorm2d(inplanes, momentum=BN_MOMENTUM),
            nn.ReLU(inplace=True),
            nn.Conv2d(inplanes, branch_planes, kernel_size=1, bias=False),
        )
        self.scale0 = nn.Sequential(
            nn.BatchNorm2d(inplanes, momentum=BN_MOMENTUM),
            nn.ReLU(inplace=True),
            nn.Conv2d(inplanes, branch_planes, kernel_size=1, bias=False),
        )
        self.scale_process = nn.Sequential(
            nn.BatchNorm2d(branch_planes * 4, momentum=BN_MOMENTUM),
            nn.ReLU(inplace=True),
            nn.Conv2d(
                branch_planes * 4,
                branch_planes * 4,
                kernel_size=3,
                padding=1,
                groups=4,
                bias=False,
            ),
        )
        self.compression = nn.Sequential(
            nn.BatchNorm2d(branch_planes * 5, momentum=BN_MOMENTUM),
            nn.ReLU(inplace=True),
            nn.Conv2d(
                branch_planes * 5, outplanes, kernel_size=1, bias=False),
        )
        self.shortcut = nn.Sequential(
            nn.BatchNorm2d(inplanes, momentum=BN_MOMENTUM),
            nn.ReLU(inplace=True),
            nn.Conv2d(inplanes, outplanes, kernel_size=1, bias=False),
        )

    @staticmethod
    def _pooled_scale(inplanes, branch_planes, kernel, stride, padding):
        return nn.Sequential(
            nn.AvgPool2d(kernel_size=kernel, stride=stride, padding=padding),
            nn.BatchNorm2d(inplanes, momentum=BN_MOMENTUM),
            nn.ReLU(inplace=True),
            nn.Conv2d(inplanes, branch_planes, kernel_size=1, bias=False),
        )

    def forward(self, inputs):
        """Aggregate parallel context scales."""
        size = inputs.shape[-2:]
        base = self.scale0(inputs)
        scales = [
            functional.interpolate(
                scale(inputs), size=size, mode='bilinear', align_corners=False)
            + base
            for scale in (self.scale1, self.scale2, self.scale3, self.scale4)
        ]
        processed = self.scale_process(torch.cat(scales, dim=1))
        return (
            self.compression(torch.cat([base, processed], dim=1))
            + self.shortcut(inputs)
        )


class PagFM(nn.Module):
    """Pixel-attention-guided fusion module."""

    def __init__(self, in_channels, mid_channels):
        super().__init__()
        self.with_channel = False
        self.after_relu = False
        self.f_x = nn.Sequential(
            nn.Conv2d(in_channels, mid_channels, kernel_size=1, bias=False),
            nn.BatchNorm2d(mid_channels),
        )
        self.f_y = nn.Sequential(
            nn.Conv2d(in_channels, mid_channels, kernel_size=1, bias=False),
            nn.BatchNorm2d(mid_channels),
        )

    def forward(self, inputs, context):
        """Fuse the P and I branches."""
        size = inputs.shape[-2:]
        context_query = functional.interpolate(
            self.f_y(context), size=size, mode='bilinear', align_corners=False)
        input_key = self.f_x(inputs)
        similarity = torch.sigmoid(
            torch.sum(input_key * context_query, dim=1).unsqueeze(1))
        context = functional.interpolate(
            context, size=size, mode='bilinear', align_corners=False)
        return (1 - similarity) * inputs + similarity * context


class LightBag(nn.Module):
    """Light boundary-attention-guided fusion module."""

    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.conv_p = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, kernel_size=1, bias=False),
            nn.BatchNorm2d(out_channels),
        )
        self.conv_i = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, kernel_size=1, bias=False),
            nn.BatchNorm2d(out_channels),
        )

    def forward(self, detail, context, boundary):
        """Fuse detail and context using boundary attention."""
        attention = torch.sigmoid(boundary)
        detail_output = self.conv_p((1 - attention) * context + detail)
        context_output = self.conv_i(context + attention * detail)
        return detail_output + context_output


class PIDNetS(nn.Module):
    """Five-class PIDNet-S inference network."""

    def __init__(self, num_classes):
        super().__init__()
        planes = 32
        ppm_planes = 96
        head_planes = 128
        self.augment = False
        self.conv1 = nn.Sequential(
            nn.Conv2d(3, planes, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(planes, momentum=BN_MOMENTUM),
            nn.ReLU(inplace=True),
            nn.Conv2d(planes, planes, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(planes, momentum=BN_MOMENTUM),
            nn.ReLU(inplace=True),
        )
        self.relu = nn.ReLU(inplace=True)
        self.layer1 = self._make_layer(BasicBlock, planes, planes, 2)
        self.layer2 = self._make_layer(
            BasicBlock, planes, planes * 2, 2, stride=2)
        self.layer3 = self._make_layer(
            BasicBlock, planes * 2, planes * 4, 3, stride=2)
        self.layer4 = self._make_layer(
            BasicBlock, planes * 4, planes * 8, 3, stride=2)
        self.layer5 = self._make_layer(
            Bottleneck, planes * 8, planes * 8, 2, stride=2)

        self.compression3 = nn.Sequential(
            nn.Conv2d(planes * 4, planes * 2, kernel_size=1, bias=False),
            nn.BatchNorm2d(planes * 2, momentum=BN_MOMENTUM),
        )
        self.compression4 = nn.Sequential(
            nn.Conv2d(planes * 8, planes * 2, kernel_size=1, bias=False),
            nn.BatchNorm2d(planes * 2, momentum=BN_MOMENTUM),
        )
        self.pag3 = PagFM(planes * 2, planes)
        self.pag4 = PagFM(planes * 2, planes)
        self.layer3_ = self._make_layer(
            BasicBlock, planes * 2, planes * 2, 2)
        self.layer4_ = self._make_layer(
            BasicBlock, planes * 2, planes * 2, 2)
        self.layer5_ = self._make_layer(
            Bottleneck, planes * 2, planes * 2, 1)

        self.layer3_d = self._make_single_layer(
            BasicBlock, planes * 2, planes)
        self.layer4_d = self._make_layer(Bottleneck, planes, planes, 1)
        self.diff3 = nn.Sequential(
            nn.Conv2d(
                planes * 4, planes, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(planes, momentum=BN_MOMENTUM),
        )
        self.diff4 = nn.Sequential(
            nn.Conv2d(
                planes * 8, planes * 2, kernel_size=3,
                padding=1, bias=False),
            nn.BatchNorm2d(planes * 2, momentum=BN_MOMENTUM),
        )
        self.spp = PAPPM(planes * 16, ppm_planes, planes * 4)
        self.dfm = LightBag(planes * 4, planes * 4)
        self.layer5_d = self._make_layer(
            Bottleneck, planes * 2, planes * 2, 1)
        self.final_layer = SegmentHead(
            planes * 4, head_planes, num_classes)

    @staticmethod
    def _make_layer(block, inplanes, planes, blocks, stride=1):
        downsample = None
        if stride != 1 or inplanes != planes * block.expansion:
            downsample = nn.Sequential(
                nn.Conv2d(
                    inplanes, planes * block.expansion,
                    kernel_size=1, stride=stride, bias=False),
                nn.BatchNorm2d(
                    planes * block.expansion, momentum=BN_MOMENTUM),
            )
        layers = [block(inplanes, planes, stride, downsample)]
        inplanes = planes * block.expansion
        for index in range(1, blocks):
            layers.append(block(
                inplanes,
                planes,
                stride=1,
                no_relu=index == blocks - 1,
            ))
        return nn.Sequential(*layers)

    @staticmethod
    def _make_single_layer(block, inplanes, planes, stride=1):
        downsample = None
        if stride != 1 or inplanes != planes * block.expansion:
            downsample = nn.Sequential(
                nn.Conv2d(
                    inplanes, planes * block.expansion,
                    kernel_size=1, stride=stride, bias=False),
                nn.BatchNorm2d(
                    planes * block.expansion, momentum=BN_MOMENTUM),
            )
        return block(inplanes, planes, stride, downsample, no_relu=True)

    def forward(self, inputs):
        """Return the main semantic logits at one-eighth resolution."""
        output_size = (inputs.shape[-2] // 8, inputs.shape[-1] // 8)
        context = self.conv1(inputs)
        context = self.layer1(context)
        context = self.relu(self.layer2(self.relu(context)))
        detail = self.layer3_(context)
        boundary = self.layer3_d(context)

        context = self.relu(self.layer3(context))
        detail = self.pag3(detail, self.compression3(context))
        boundary = boundary + functional.interpolate(
            self.diff3(context), size=output_size,
            mode='bilinear', align_corners=False)

        context = self.relu(self.layer4(context))
        detail = self.layer4_(self.relu(detail))
        boundary = self.layer4_d(self.relu(boundary))
        detail = self.pag4(detail, self.compression4(context))
        boundary = boundary + functional.interpolate(
            self.diff4(context), size=output_size,
            mode='bilinear', align_corners=False)

        detail = self.layer5_(self.relu(detail))
        boundary = self.layer5_d(self.relu(boundary))
        context = functional.interpolate(
            self.spp(self.layer5(context)), size=output_size,
            mode='bilinear', align_corners=False)
        return self.final_layer(self.dfm(detail, context, boundary))


def build_pidnet_s(num_classes):
    """Build the deployment PIDNet-S inference graph."""
    return PIDNetS(num_classes=num_classes)
