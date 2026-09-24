// SPDX-License-Identifier: GPL-2.0-only
/*
 * psp2sgx: DRM driver for the PowerVR SGX543MP4+ of the PlayStation Vita.
 *
 * The GPU runs a microkernel this driver assembles at probe time
 * (psp2sgx_mk.c) instead of Imagination's or Sony's firmware.  The driver
 * exposes a render node only: buffer objects in a GPU address space per open
 * file, and jobs made of checked register operations, scheduled with
 * drm_sched.  Userspace (Mesa's psp2sgx gallium driver) compiles shaders to
 * USSE and PDS code and builds the vertex and render state.
 */
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_ioctl.h>
#include <drm/psp2sgx_drm.h>

#include "psp2sgx_drv.h"
#include "psp2sgx_gem.h"

/* psp2sgx.enable=0 on the kernel command line keeps the GPU off */
static bool psp2sgx_enable = true;
module_param_named(enable, psp2sgx_enable, bool, 0444);
MODULE_PARM_DESC(enable, "Power up and drive the GPU (default: true)");

static int psp2sgx_ioctl_get_param(struct drm_device *drm, void *data,
				   struct drm_file *file)
{
	struct psp2sgx_device *sgx = to_psp2sgx(drm);
	struct drm_psp2sgx_get_param *args = data;

	if (args->pad)
		return -EINVAL;
	switch (args->param) {
	case DRM_PSP2SGX_PARAM_UAPI_VERSION:
		args->value = PSP2SGX_UAPI_VERSION;
		break;
	case DRM_PSP2SGX_PARAM_CORE_ID:
		args->value = sgx->core_id;
		break;
	case DRM_PSP2SGX_PARAM_CORE_REVISION:
		args->value = sgx->core_revision;
		break;
	case DRM_PSP2SGX_PARAM_NUM_CORES:
		args->value = PSP2SGX_NUM_CORES;
		break;
	case DRM_PSP2SGX_PARAM_GENERAL_HEAP_BASE:
		args->value = PSP2SGX_GENERAL_BASE;
		break;
	case DRM_PSP2SGX_PARAM_GENERAL_HEAP_SIZE:
		args->value = PSP2SGX_GENERAL_END - PSP2SGX_GENERAL_BASE;
		break;
	case DRM_PSP2SGX_PARAM_CODE_VERTEX_BASE:
		args->value = PSP2SGX_CODE_VERTEX_BASE;
		break;
	case DRM_PSP2SGX_PARAM_CODE_FRAGMENT_BASE:
		args->value = PSP2SGX_CODE_FRAGMENT_BASE;
		break;
	case DRM_PSP2SGX_PARAM_CODE_WINDOW_SIZE:
		args->value = PSP2SGX_CODE_WINDOW;
		break;
	case DRM_PSP2SGX_PARAM_PDS_HEAP_BASE:
		args->value = PSP2SGX_PDS_BASE;
		break;
	case DRM_PSP2SGX_PARAM_PDS_HEAP_SIZE:
		args->value = PSP2SGX_PDS_END - PSP2SGX_PDS_BASE;
		break;
	case DRM_PSP2SGX_PARAM_MAX_REG_OPS:
		args->value = PSP2SGX_MAX_REG_OPS;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int psp2sgx_open(struct drm_device *drm, struct drm_file *file)
{
	struct psp2sgx_device *sgx = to_psp2sgx(drm);
	struct drm_gpu_scheduler *sched = &sgx->sched;
	struct psp2sgx_file *pf;
	int err;

	pf = kzalloc(sizeof(*pf), GFP_KERNEL);
	if (!pf)
		return -ENOMEM;
	pf->vm = psp2sgx_vm_create(sgx);
	if (IS_ERR(pf->vm)) {
		err = PTR_ERR(pf->vm);
		goto err_free;
	}
	err = drm_sched_entity_init(&pf->entity, DRM_SCHED_PRIORITY_NORMAL,
				    &sched, 1, NULL);
	if (err)
		goto err_vm;
	file->driver_priv = pf;
	return 0;

err_vm:
	psp2sgx_vm_put(pf->vm);
err_free:
	kfree(pf);
	return err;
}

static void psp2sgx_postclose(struct drm_device *drm, struct drm_file *file)
{
	struct psp2sgx_file *pf = file->driver_priv;

	drm_sched_entity_destroy(&pf->entity);
	psp2sgx_vm_put(pf->vm);
	kfree(pf);
}

static const struct drm_ioctl_desc psp2sgx_ioctls[] = {
	DRM_IOCTL_DEF_DRV(PSP2SGX_GET_PARAM, psp2sgx_ioctl_get_param, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(PSP2SGX_BO_CREATE, psp2sgx_ioctl_bo_create, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(PSP2SGX_BO_INFO, psp2sgx_ioctl_bo_info, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(PSP2SGX_BO_WAIT, psp2sgx_ioctl_bo_wait, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(PSP2SGX_SUBMIT, psp2sgx_ioctl_submit, DRM_RENDER_ALLOW),
};

DEFINE_DRM_GEM_FOPS(psp2sgx_fops);

static const struct drm_driver psp2sgx_driver = {
	.driver_features = DRIVER_RENDER | DRIVER_GEM | DRIVER_SYNCOBJ,
	.open = psp2sgx_open,
	.postclose = psp2sgx_postclose,
	.ioctls = psp2sgx_ioctls,
	.num_ioctls = ARRAY_SIZE(psp2sgx_ioctls),
	.fops = &psp2sgx_fops,
	.name = "psp2sgx",
	.desc = "PlayStation Vita SGX543MP4+",
	.major = 1,
	.minor = 0,
	.gem_create_object = psp2sgx_gem_create_object,
	.gem_prime_import_sg_table = drm_gem_shmem_prime_import_sg_table,
};

static irqreturn_t psp2sgx_irq(int irq, void *data)
{
	struct psp2sgx_device *sgx = data;
	u32 status = sgx_read(sgx, SGX_EVENT_STATUS);

	if (!(status & SGX_EVENT_SW_EVENT))
		return IRQ_NONE;
	sgx_write(sgx, SGX_EVENT_HOST_CLEAR, SGX_EVENT_SW_EVENT);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t psp2sgx_irq_thread(int irq, void *data)
{
	psp2sgx_job_irq(data);
	return IRQ_HANDLED;
}

static int psp2sgx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct psp2sgx_device *sgx;
	u32 smc[2];
	int err;

	if (!psp2sgx_enable)
		return -ENODEV;

	sgx = devm_drm_dev_alloc(dev, &psp2sgx_driver, struct psp2sgx_device, drm);
	if (IS_ERR(sgx))
		return PTR_ERR(sgx);
	sgx->dev = dev;
	mutex_init(&sgx->hw_lock);
	psp2sgx_diag_init(sgx);
	platform_set_drvdata(pdev, sgx);

	sgx->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sgx->regs))
		return PTR_ERR(sgx->regs);
	sgx->irq = platform_get_irq(pdev, 0);
	if (sgx->irq < 0)
		return sgx->irq;

	/* sony,power-smc = <service bus>: switch the power domain through the monitor */
	sgx->power_smc = -1;
	if (!of_property_read_u32_array(dev->of_node, "sony,power-smc", smc, 2)) {
		sgx->power_smc = smc[0];
		sgx->power_smc_bus = smc[1];
	}

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (err)
		return err;

	err = psp2sgx_hw_power_on(sgx);
	if (err)
		return err;
	dev_info(dev, "SGX core id %#x, revision %#x\n", sgx->core_id,
		 sgx->core_revision);

	err = psp2sgx_kregion_init(sgx);
	if (err)
		goto err_power;
	err = psp2sgx_hw_build_kregion(sgx);
	if (err)
		goto err_kregion;
	err = psp2sgx_hw_init(sgx);
	if (err)
		goto err_kregion;

	err = psp2sgx_sched_init(sgx);
	if (err)
		goto err_kregion;

	err = devm_request_threaded_irq(dev, sgx->irq, psp2sgx_irq,
					psp2sgx_irq_thread, IRQF_SHARED,
					dev_name(dev), sgx);
	if (err)
		goto err_sched;

	/* a failed self-test is logged; jobs will time out with a register dump */
	psp2sgx_diag_selftest(sgx);

	err = drm_dev_register(&sgx->drm, 0);
	if (err)
		goto err_sched;
	return 0;

err_sched:
	psp2sgx_sched_fini(sgx);
	psp2sgx_vm_put(sgx->current_vm);
	sgx->current_vm = NULL;
err_kregion:
	psp2sgx_kregion_fini(sgx);
err_power:
	psp2sgx_hw_power_off(sgx);
	return err;
}

static void psp2sgx_remove(struct platform_device *pdev)
{
	struct psp2sgx_device *sgx = platform_get_drvdata(pdev);

	drm_dev_unregister(&sgx->drm);
	psp2sgx_diag_fini(sgx);
	psp2sgx_sched_fini(sgx);
	psp2sgx_vm_put(sgx->current_vm);
	sgx->current_vm = NULL;
	psp2sgx_hw_power_off(sgx);
	devm_free_irq(sgx->dev, sgx->irq, sgx);
	psp2sgx_kregion_fini(sgx);
}

static const struct of_device_id psp2sgx_of_match[] = {
	{ .compatible = "sony,vita-gpu" },
	{ }
};
MODULE_DEVICE_TABLE(of, psp2sgx_of_match);

static struct platform_driver psp2sgx_platform_driver = {
	.probe = psp2sgx_probe,
	.remove = psp2sgx_remove,
	.driver = {
		.name = "psp2sgx",
		.of_match_table = psp2sgx_of_match,
	},
};
module_platform_driver(psp2sgx_platform_driver);

MODULE_DESCRIPTION("PlayStation Vita SGX543MP4+ DRM driver");
MODULE_LICENSE("GPL");
