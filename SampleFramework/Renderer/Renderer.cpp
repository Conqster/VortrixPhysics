#include <SampleFramework/SampleFramework.h>

#include "Renderer.h"

#include "Texture.h"
#include "SampleFramework/Camera.h"

#include "SampleFramework/Utils/Util.h"
#include "SampleFramework/Utils/Loader.h"

#include "SampleFramework/Input/InputSystem.h"

#include "Vortrix/Core/Profiler.h"

#include "Vortrix/Maths/ViewProjection.h"




RendererImpl::RendererImpl(ApplicationWindow* display_window)
{
	Initialise(display_window);
}

void RendererImpl::Initialise(ApplicationWindow* display_window)
{
	mWindow = display_window;

	
	bool success = mShadowShader.Create("shadow_depth", "assets/shaders/shadowDepth.vert", "assets/shaders/shadowDepth.frag");
	//success &= mWorldGridShader.Create("world_grid", "assets/shaders/worldGrid/worldGrid.vert", "assets/shaders/worldGrid/worldGrid.frag", "assets/shaders/worldGrid/worldGrid.geo");
	//success &= mTex2ScreenShader.Create("texture_screen", "assets/shaders/TextureToScreen.vert", "assets/shaders/TextureToScreen.frag");

	VX_ASSERT_WARN(success, "Failed create a shaders!!!");



	mPlainTexture = TextureFactory::CreateFromFile("assets/textures/plain64.png", true, "White");
	if (!mPlainTexture->IsValid()) ///generate a plain texture data
	{
		uint32 w = 64;
		uint32 h = w;

		TextureFormat internal_format = TextureFormat::RGBA8;
		PixelFormat px_format = PixelFormat::RGBA;
		PixelType px_type = PixelType::UByte;
		
		uint32 channel_count = PixelFormatChannelCount(px_format);

		std::vector<uint8> px(w * h * channel_count, 255);

		mPlainTexture->CreateFromPixel((const void*)px.data(),
			{ w, h, internal_format, px_format, px_type, true, "Plain white texture"});
	}
	if (!(mBrickTexture = TextureFactory::CreateFromFile("assets/textures/floor_brick/patterned_brick_floor_diff.jpg", true, "Brick"))->IsValid())
	{
		VX_ASSERT_WARN("brick sharing plain texture");
		mBrickTexture = mPlainTexture;
	}
	mCheckersTexture = TextureFactory::CreateFromFile("assets/textures/test_checkers.jpg", true, "Checkers Texture");

	mLinearRepeatSampler = new Sampler(SamplerCreateInfo());
	mLinearRepeatSampler = new Sampler({ TextureWrap::ClampToEdge, TextureFilter::Linear });
	const float border[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	//const float border[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
	mNearestClampBorderSampler = new Sampler({ TextureWrap::ClampToBorder, TextureFilter::Nearest, border });

	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	//mCubePrimitive = Loader::LoadMesh("assets/meshes/cube.rmesh");
	mQuadPrimitive = Loader::LoadMesh("assets/meshes/quad.rmesh");
	//mQuadXZPrimitive = Loader::LoadMesh("assets/meshes/quadXZ.rmesh");
	//mSpherePrimitive = Util::CreateSphere(); 
	//mCapsulePrimitive = Util::CreateCapsule(1.0f, 0.5f, 3);
	//mSpherePrimitive = Util::CreateOctaSphere();
	//mSpherePrimitive = Util::CreateCapsule(1.0f, 0.5f, 3);
	//mTrianglePrimitive = Loader::LoadMesh("assets/meshes/triangle.rmesh");

	mShadowMapRT.Create(4096, 4096);

	TextureCreateInfo ci;
	ci.width = 256/2;// mWindow->Width();
	ci.height = 256/2;// mWindow->Height();
	ci.internalFormat = TextureFormat::RGBA16F;// TextureFormat::SRGBA8;
	ci.pxFormat = PixelFormat::RGBA;
	ci.pxType = PixelType::Float;// PixelType::UByte;
	ci.name = "Test Generic RT";
	mTestRt.Create({ci}, {ci.width, ci.height, TextureFormat::Depth24Stencil8});
	//mTestRt.Create({ci}, {});
	ci.width = ci.height = 256;
	ci.internalFormat = TextureFormat::RGB8;// TextureFormat::SRGBA8;
	ci.pxFormat = PixelFormat::RGB;
	ci.pxType = PixelType::UByte;
	ci.name = "Directional Light Debug RT";
	mDirLightDebugRT.Create({ci}, {ci.width, ci.height, TextureFormat::Depth24Stencil8});


	SetCallbacks();


	mFont = vx::MakeRef<Font>();
	mFontShader = vx::MakeRef<Shader>();
	mFontShader->Create("Font shader", "assets/shaders/FontShader.vert", "assets/shaders/FontShader.frag");
	mFont->Create("assets/fonts/Inter_18pt-Regular.ttf", mFontShader);

	GenerateBoxGeometry();
	//GenerateSphereGeometry();
	//CreateSphereOctant();
	CreateSphere2(0.5f, 3);
	CreateCapsuleGeometry(0.5f, 0.5f, 3);
	mQuadXZGeometry = LoadMeshAsGeometry("assets/meshes/quadXZ.rmesh");
	mQuadXZGeometry->AssignGPULabel("Quad XZ Geometry");

	mQuadXZDoubledSidedGeometry = mQuadXZGeometry->Instance();

	mQuadGeometry = LoadMeshAsGeometry("assets/meshes/quad.rmesh");
	mQuadGeometry->AssignGPULabel("Quad Geometry");
}

void RendererImpl::DrawText3D(const std::string_view& text, const vx::Vec3& pos, float scale, const vx::Colour& col, ETextAlignment align)
{
	mFont->DrawText3D(text, pos, mCamera->GetRight(), mCamera->Up(), scale, col, align);
}


void RendererImpl::DrawText3D_DynScale(const std::string_view& text, const vx::Vec3& pos, float scale, const vx::Colour& col, ETextAlignment align)
{
	float dist = vx::VxAbs((mCamera->Position() - pos).Length());
	scale *= dist;
	mFont->DrawText3D(text, pos, mCamera->GetRight(), mCamera->Up(), scale, col, align);
}

void RendererImpl::BeginFrame(Camera* p_camera, vx::Colour clear_colour)
{
	VX_VARIABLE_PROFILE_FUNCTION();

	glClearColor(clear_colour.R(), clear_colour.G(), clear_colour.B(), clear_colour.A());
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	//glEnable(GL_STENCIL_TEST);

	//glStencilFunc(GL_ALWAYS, 1, 0xFF);
	//glStencilMask(0xFF);
	//glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);



	mCamera = p_camera;

	if (!p_camera)
	{
		EndFrame();
		return;
	}


	//shadow properties 
	//calculate light project view matrix 
	vx::Vec3 light_dir = mDirLight.direction.Normalised();
	light_dir = (light_dir.IsZero()) ? vx::Vec3(-0.2f).Normalised() : light_dir;


	mFontShader->Bind();
	mFontShader->SetUniformMat4("uProjection", mCamera->ProjMat(mWindow->AspectRatio()));
	mFontShader->SetUniformMat4("uView", mCamera->ViewMat());


	mGPUShadowData.zfar = mCamera->GetProperties().zFar;
	mGPUShadowData.viewProj = mShadowData.ComputeProjectionViewMat(light_dir);

	mDirLightShadowUBO.Upload(&mGPUShadowData);

}

void RendererImpl::EndFrame()
{
	VX_VARIABLE_PROFILE_FUNCTION();
	//mDisplay.FlushAndSwapBuffer();
	mCamera = nullptr;
	//mFrameEntitiesCount = 0;
	//memset(&mFrameRenderableEntities, {}, sizeof(mFrameRenderableEntities));

	ClearGeometriesInstances();
}



void RendererImpl::Destroy()
{
	Camera* mCamera = nullptr;

	mBrickTexture->Destroy();
//	delete mBrickTexture;
	mBrickTexture = nullptr;

	mCheckersTexture->Destroy();
	//delete mCheckersTexture;
	mCheckersTexture = nullptr;

	mLinearRepeatSampler->Destroy();
	//delete mLinearRepeatSampler;
	mLinearRepeatSampler = nullptr;

	mWindow->SetWindowResizeListener({});
}

void RendererImpl::ShadowPass()
{
	VX_VARIABLE_PROFILE_FUNCTION();
	//quick hack test
	glCullFace(GL_FRONT);
	mShadowShader.Bind();
	mShadowShader.SetUniformMat4("uLightSpaceMat", mGPUShadowData.viewProj);

	mShadowMapRT.Bind();
	glClear(GL_DEPTH_BUFFER_BIT);
	//Draw avaliable mesh geometry
	DrawObjects(mShadowShader, true);


	mInstanceShadowDepthShader->Bind();
	mInstanceShadowDepthShader->SetUniformMat4("uLightSpaceMat", mGPUShadowData.viewProj);
	//draw geometries instances
	RenderGeometriesInstances(true);
	Shader::UnbindProgram();

	//swicth back to default frame buffer
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glCullFace(GL_BACK);
	glViewport(0, 0, mWindow->Width(), mWindow->Height());
}

void RendererImpl::DrawPass()
{
	VX_VARIABLE_PROFILE_FUNCTION();
	//Draw avaliable mesh geometry
	//DrawObjects(mMeshShader);


	//draw geometries instances
	mGeometryShader->Bind();
	RenderGeometriesInstances();
	Shader::UnbindProgram();


	////DRAW FONTS 
	//glBindFramebuffer(GL_FRAMEBUFFER, 0);
	//glViewport(0, 0, mWindow->Width(), mWindow->Height());
	DisableDepth();
	DepthWriteMask(false);
	mFont->DrawFrame();
	DepthWriteMask(true);
	EnableDepth();




	mTestRt.Bind();
	vx::Colour col = vx::Colour::sBlue;
	//glClearColor(col.R(), col.G(), col.B(), col.A());
	glClearColor(col.R(), col.G(), col.B(), 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);




	//Draw avaliable mesh geometry
	//DrawObjects(mMeshShader);

	//swicth back to default frame buffer
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	//glViewport(0, 0, mWindow->Width(), mWindow->Height());
	glViewport(0, 0, mWindow->Width() * 0.4f, mWindow->Height() * 0.4f);

	//glEnable(GL_STENCIL_TEST);
	//glStencilFunc(GL_NOTEQUAL, 1, 0xFF);
	//glStencilMask(0x00);

	//glDisable(GL_DEPTH_TEST);

	//mTex2ScreenShader.Bind();
	//mTex2ScreenShader.SetUniform1i("uTexture", 0);
	mTestRt.BindAttachment(0, 0);
	//mQuadPrimitive.DrawOutline();
	//mQuadPrimitive.Draw();
	//glViewport(0, 0, mWindow->Width(), mWindow->Height());
	
	//glStencilMask(0xFF);
	//glStencilFunc(GL_ALWAYS, 1, 0xFF);
	//glDisable(GL_STENCIL_TEST);
	//glEnable(GL_DEPTH_TEST);




}

void RendererImpl::SetCallbacks()
{

	///Window Resize callback 
	mWindow->SetWindowResizeListener([this](uint32_t w, uint32_t h) { OnResize(w, h);});


	//This is not required OrthoQuery set corners in world
	//mShadowData.SetOriginQuery([this](ShadowData* shadow_data) {


	//			float z_near = mCamera->GetProperties().zNear;
	//	float z_far = mCamera->GetProperties().zFar;

	//	float split_near = z_near; //prev_cascade_far;

	//	float split_far = z_near + (z_far - z_near) * shadow_data->split_depth;
	//	float split_center = (split_near + split_far) * 0.5f;
	//	return mCamera->Position() + mCamera->Forward() * split_center;


	//	});

//	mShadowData.SetOriginQuery([this](ShadowData* shadow) {ShadowOriginQuery(shadow); });


	mShadowData.SetOrthoQuery([this](const vx::Mat44& light_view, ShadowData* shadow_data) {

		std::function<void(vx::Vec3 out_corner[8], const vx::Mat44& vp)> GetFrustum =
			[&](vx::Vec3 out_corner[8], const vx::Mat44& vp) {

			vx::Mat44 inv_vp = vp.Inverse();

			static const vx::Vec3 ndc[8] = {
				{-1, -1, -1},
				{1, -1, -1},
				{-1, 1, -1},
				{1, 1, -1},
				{-1, -1, 1},
				{1, -1, 1},
				{-1, 1, 1},
				{1, 1, 1}
			};

			for (int i = 0; i < 8; i++)
			{
				vx::Vec4 corner = inv_vp.Multiply(vx::Vec4(ndc[i], 1.0f));
				out_corner[i] = (corner.XYZ() / corner.W());
			}
			};

		std::function<void(const vx::Vec3 corners[8], const vx::Vec3& light_dir,
			vx::Vec3& o_min, vx::Vec3& o_max, vx::Vec3& o_center)> GetLightBounds =
			[&](const vx::Vec3 corners[8], const vx::Vec3& light_dir,
				vx::Vec3& o_min, vx::Vec3& o_max, vx::Vec3& o_center) {


					vx::Vec3 light_corners[8];
					for (int i = 0; i < 8; i++)
						light_corners[i] = light_view.Transform(corners[i]);

					//AABB
					o_min = light_corners[0];
					o_max = light_corners[0];
					for (int i = 1; i < 8; i++)
					{
						o_min = vx::Vec3::Min(o_min, light_corners[i]);
						o_max = vx::Vec3::Max(o_max, light_corners[i]);
					}

					o_center = (o_min + o_max) * 0.5f;
			};


		vx::Vec3 corners[8];
		const auto& cam_prop = mCamera->GetProperties();
		float z_near = cam_prop.zNear;
		float z_far = cam_prop.zFar;
		float split_far = z_near + (z_far - z_near) * shadow_data->split_depth;
		float split_near = z_near + shadow_data->zNearOffset;
		vx::Mat44 proj = vx::Perspective(vx::DegToRad(cam_prop.fovY), mWindow->AspectRatio(), split_near, split_far);
		vx::Mat44 vp = proj.Multiply(mCamera->ViewMat());

		GetFrustum(corners, vp);



		//vx::Vec3 aabb_min, aabb_max, aabb_center;
		vx::Vec3 aabb_center;
		auto& aabb = shadow_data->orthographicBounds;
		GetLightBounds(corners, mDirLight.direction.Normalised(), aabb.mMin, aabb.mMax, aabb_center);

		float extend = (split_far - split_near) * 0.5f;
		});
}

void RendererImpl::DrawObjects(Shader& shader, bool only_depth)
{
	shader.Bind();
	//if (mFrameEntitiesCount <= 0)
	//	return;

	if (!only_depth)
	{
		//all unit needs this sampler 
		// 0 -> model texture
		// 1 -> shadow map sampling
		mLinearRepeatSampler->Bind(0);
		//mLinearRepeatSampler->Bind(1);
		
		//mShadowMapFBO.Read(1);
		mNearestClampBorderSampler->Bind(1);
		mShadowMapRT.BindAttachment(0, 1);
		//mPlainTexture->Bind(1);
		shader.SetUniform1i("uShadowMap", 1);
		
		if (mCheckersTexture)
			mCheckersTexture->Bind(0);

		glEnable(GL_BLEND);
		glEnable(GL_DEPTH_TEST);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}


	//int use_idx = (mFrameRenderableEntities.size() > 5) ? 5 : 1;
	//RenderableEntity entt = mFrameRenderableEntities[use_idx];
	//vx::Mat44 transform = entt.transform;
	//transform.SetTranslation(vx::Vec3(1.0f, 1.5f, 1.0f));
	//shader.SetUniformMat4("uModel", transform);
	//if(!only_depth)
	//	shader.SetUniformVec4("uColour", entt.colour);
	//DrawGeometry(entt.transform, vx::Colour::sBlack, mBoxGeometry);

	//for (unsigned int i = 0; i < mFrameEntitiesCount; i++)
	//{
	//	if (only_depth && !mFrameRenderableEntities[i].canCastShadow)
	//		continue;

	//	RenderableEntity entt = mFrameRenderableEntities[i];
	//	shader.SetUniformMat4("uModel", entt.transform);

	//	bool used_plain_texture = false;
	//	if (!only_depth)
	//	{
	//		if (entt.plainTexture && mPlainTexture)
	//		{
	//			mPlainTexture->Bind();
	//			used_plain_texture = true;
	//		}

	//		shader.SetUniformVec4("uColour", entt.colour);
	//	}
	//	

	//	if (entt.solidRender)
	//		entt.renderableMesh->Draw();
	//	else
	//		entt.renderableMesh->DrawOutline();

	//	//reset texture
	//	if (used_plain_texture && mCheckersTexture)
	//		mCheckersTexture->Bind();
	//}

	if(!only_depth)
		glDisable(GL_BLEND);



}

//void RendererImpl::AddFrameRenderableEntity(const RenderableEntity entity)
//{
//	if (mFrameEntitiesCount < mMaxFrameEntity)
//	{
//		mFrameRenderableEntities[mFrameEntitiesCount++] = entity;
//	}
//}

void RendererImpl::RenderGeometriesInstances(bool only_depth)
{
	if(!only_depth)
	{
		struct CamHackData
		{
			GizmosCameraData cam;
			vx::Vec4 pos;
		};

		CamHackData cam_data =
		{
			{mCamera->ProjMat(mWindow->AspectRatio()), mCamera->ViewMat()},
			vx::Vec4(mCamera->Position(),0.0f)
		};


		mCameraUBO.Upload(&cam_data);
		mCameraUBO.Bind();
		vx::Vec3 light_dir = mDirLight.direction.Normalised();
		light_dir = (light_dir.IsZero()) ? vx::Vec3(-0.2f).Normalised() : light_dir;
		static GPULight old_gpu_light;
		GPULight gpu_light = GPULight(light_dir.ToFloat3(), 0.2f, mDirLight.colour);

		if (gpu_light != old_gpu_light)
		{
			old_gpu_light = gpu_light;
			mDirLightUBO.Upload(&gpu_light);
		}

		mNearestClampBorderSampler->Bind(1);
		mShadowMapRT.BindAttachment(0, 1);
		//mPlainTexture->Bind(1);
		mGeometryShader->SetUniform1i("uFallbackShadowMap", 1);
		if (!ApplicationWindow::SupportsBindless())
		{
			mLinearRepeatSampler->Bind(0);
			mPlainTexture->Bind(0);
			//mPlainTexture->Bind(1);
			mGeometryShader->SetUniform1i("uFallback", 0);
		}


		glEnable(GL_BLEND);
		glEnable(GL_DEPTH_TEST);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}


	static bool first = true;

	for (auto& [ref_geometry, instances] : mSolidGeometries)
	{
		if (instances.buffer.empty())
			continue;

		if(instances.isDirty)
		{

			size_t required_bytes = instances.buffer.size() * sizeof(Instance);

			if(first)
			required_bytes = 512 * sizeof(Instance);

			mInstanceSSBO.ValidateSize(required_bytes);

			//mInstanceSSBO.Upload(instances.data());
			mInstanceSSBO.SegmentUpload(instances.buffer.data(), required_bytes);
		//	instances.isDirty = false;
		}
		
		//draw
		if (!only_depth && instances.disableCulling)
		{
			glDisable(GL_CULL_FACE);
			if(instances.wireFrame)
				ref_geometry->GetBatch()->DrawWireFrameInstanced(instances.buffer.size());
			else
				ref_geometry->GetBatch()->DrawInstances(instances.buffer.size());
			glEnable(GL_CULL_FACE);
		}
		else
		{
			if(instances.wireFrame)
				ref_geometry->GetBatch()->DrawWireFrameInstanced(instances.buffer.size());
			else
				ref_geometry->GetBatch()->DrawInstances(instances.buffer.size());
		}
		//auto ref_tri_batch = ref_geometry->GetBatch();
		//ref_tri_batch->BindArray();
		////no need to bind the buffer only is write/upload is needed
		////glDrawArrays(GL_TRIANGLES, 0, ref_tri_batch->count);
		//glDrawArraysInstanced(GL_TRIANGLES, 0, ref_tri_batch->count, instances.buffer.size());

	}


	first = false;

	if (!only_depth)
		glDisable(GL_BLEND);

}

