#include "Logic.h"
#include <fstream>
//#define _ALL_VISIBLE_

extern const bool asynchronous;

//辅助函数
std::shared_ptr<THUAI4::Character> obj2C(const Protobuf::GameObjInfo& goi)
{
	std::shared_ptr<THUAI4::Character> character = std::make_shared<THUAI4::Character>();
	character->ap = goi.ap();
	character->bulletNum = goi.bulletnum();
	character->bulletType = (THUAI4::BulletType)goi.bullettype();
	character->CD = goi.cd();
	character->facingDirection = goi.facingdirection();
	character->guid = goi.guid();
	character->hp = goi.hp();
	character->isDying = goi.isdying();
	character->isMoving = goi.ismoving();
	character->jobType = (THUAI4::JobType)goi.jobtype();
	character->lifeNum = goi.lifenum();
	character->maxBulletNum = goi.maxbulletnum();
	character->maxHp = goi.maxhp();
	character->moveSpeed = goi.movespeed();
	character->propType = (THUAI4::PropType)goi.proptype();
	character->radius = goi.radius();
	character->shapeType = (THUAI4::ShapeType)goi.shapetype();
	character->teamID = static_cast<uint16_t>(goi.teamid());
	character->x = goi.x();
	character->y = goi.y();
	return character;
}
std::shared_ptr<THUAI4::Wall> obj2W(const Protobuf::GameObjInfo& goi)
{
	std::shared_ptr<THUAI4::Wall> wall = std::make_shared<THUAI4::Wall>();
	wall->guid = goi.guid();
	wall->radius = goi.radius();
	wall->shapeType = (THUAI4::ShapeType)goi.shapetype();
	wall->x = goi.x();
	wall->y = goi.y();
	return wall;
}
std::shared_ptr<THUAI4::Prop> obj2P(const Protobuf::GameObjInfo& goi)
{
	std::shared_ptr<THUAI4::Prop> prop = std::make_shared<THUAI4::Prop>();
	prop->facingDirection = goi.facingdirection();
	prop->guid = goi.guid();
	prop->isLaid = goi.islaid();
	prop->isMoving = goi.ismoving();
	prop->moveSpeed = goi.movespeed();
	prop->propType = (THUAI4::PropType)goi.proptype();
	prop->radius = goi.radius();
	prop->shapeType = (THUAI4::ShapeType)goi.shapetype();
	prop->x = goi.x();
	prop->y = goi.y();
	return prop;
}
std::shared_ptr<THUAI4::Bullet> obj2Blt(const Protobuf::GameObjInfo& goi)
{
	std::shared_ptr<THUAI4::Bullet> bullet = std::make_shared<THUAI4::Bullet>();
	bullet->ap = goi.ap();
	bullet->bulletType = (THUAI4::BulletType)goi.bullettype();
	bullet->facingDirection = goi.facingdirection();
	bullet->guid = goi.guid();
	bullet->isMoving = goi.ismoving();
	bullet->moveSpeed = goi.movespeed();
	bullet->radius = goi.radius();
	bullet->shapeType = (THUAI4::ShapeType)goi.shapetype();
	bullet->teamID = static_cast<uint16_t>(goi.teamid());
	bullet->x = goi.x();
	bullet->y = goi.y();
	return bullet;
}
std::shared_ptr<THUAI4::BirthPoint> obj2Bp(const Protobuf::GameObjInfo& goi)
{
	std::shared_ptr<THUAI4::BirthPoint> birthpoint = std::make_shared<THUAI4::BirthPoint>();
	birthpoint->guid = goi.guid();
	birthpoint->radius = goi.radius();
	birthpoint->shapeType = (THUAI4::ShapeType)goi.shapetype();
	birthpoint->teamID = static_cast<uint16_t>(goi.teamid());
	birthpoint->x = goi.x();
	birthpoint->y = goi.y();
	return birthpoint;
}
bool visible(int32_t x, int32_t y, Protobuf::GameObjInfo& g)
{
	Protobuf::PropType pT = g.proptype();
	if (g.islaid() && (pT == Protobuf::PropType::Attenuator || pT == Protobuf::PropType::Dirt || pT == Protobuf::PropType::Divider))
		return false;

	int64_t dx = x - g.x();
	int64_t dy = y - g.y();
	uint64_t distanceSquared = dx * dx + dy * dy;
	return distanceSquared <= Constants::Map::sightRadiusSquared;
}

void Logic::ProcessM2C(std::shared_ptr<Protobuf::MessageToClient> pM2C)
{
	if (pM2C->playerid() != ID::GetPlayerID() || pM2C->teamid() != ID::GetTeamID())
		return; //权宜之计
	switch (pM2C->messagetype())
	{
	case Protobuf::MessageType::StartGame:
	{
		load(pM2C);

		//playerGuid只在这里记录
		for (int i = 0; i < pM2C->playerguids_size() && pBuffer->playerGUIDs.size(); i++)
		{
			auto temp = pM2C->playerguids(i);
			for (int j = 0; j < temp.teammateguids_size() && pBuffer->playerGUIDs[i].size(); j++)
			{
				State::playerGUIDs[i][j] = static_cast<int64_t>(pM2C->playerguids(i).teammateguids(j));
			}
		}

		gamePhase = GamePhase::Gaming;
		std::cout << "游戏开始" << std::endl;
		std::thread tAI(asynchronous ? &Logic::PlayerWrapperAsyn : &Logic::PlayerWrapper, this);
		tAI.detach();//虽在这里detach了，还是要在Main末尾同步
		break;
	}
	case Protobuf::MessageType::Gaming:
		load(pM2C);
		break;

	case Protobuf::MessageType::EndGame:
	{
		gamePhase = GamePhase::GameOver;
		std::cout << "Game ends\n";
		{
			std::lock_guard<std::mutex> lck(mtx_buffer);
			FlagBufferUpdated = true;
			counter_buffer = -1;
		}
		cv_buffer.notify_one();
		break;
	}

	default:
		std::cout << "Invalid MessageType wrt M2C" << std::endl;
	}
}

void Logic::ProcessM2OC(std::shared_ptr<Protobuf::MessageToOneClient> pM2OC)
{
	switch (pM2OC->messagetype())
	{
	case Protobuf::MessageType::ValidPlayer:
		std::cout << "Valid player." << std::endl;
		break;
	case Protobuf::MessageType::InvalidPlayer:
		gamePhase = GamePhase::GameOver;
		std::cout << "Invalid player!" << std::endl;
		break;
	case Protobuf::MessageType::Send:
		MessageStorage.push(pM2OC->message());
		break;
	default:
		std::cout << "Invalid MessageType wrt M2OC" << std::endl;
	}
}

void Logic::load(std::shared_ptr<Protobuf::MessageToClient> pM2C)
{
	{
		//首先load到buffer
		std::lock_guard<std::mutex> lck(mtx_buffer);
		pBuffer->characters.clear();
		pBuffer->walls.clear();
		pBuffer->props.clear();
		pBuffer->bullets.clear();
		pBuffer->birthpoints.clear();
#ifdef _COLOR_MAP_BY_HASHING_
		pBuffer->cellColors.clear();
#endif // _COLOR_MAP_BY_HASHING_
		pBuffer->teamScore = pM2C->teamscore();
		pBuffer->selfTeamColor = (THUAI4::ColorType)pM2C->selfteamcolor();
		pBuffer->self = obj2C(pM2C->selfinfo());
		int selfX = pBuffer->self->x;
		int selfY = pBuffer->self->y;

		for (auto it : pM2C->gameobjs())
		{
			if (
#ifdef _ALL_VISIBLE_
				true
#else
				visible(selfX, selfY, it)
#endif // _ALL_VISIBLE_
				)
			{
				switch (it.gameobjtype())
				{
				case Protobuf::GameObjType::Character:
					pBuffer->characters.push_back(obj2C(it));
					break;
				case Protobuf::GameObjType::Wall:
					pBuffer->walls.push_back(obj2W(it));
					break;
				case Protobuf::GameObjType::Prop:
					pBuffer->props.push_back(obj2P(it));
					break;
				case Protobuf::GameObjType::Bullet:
					pBuffer->bullets.push_back(obj2Blt(it));
					break;
				case Protobuf::GameObjType::BirthPoint:
					pBuffer->birthpoints.push_back(obj2Bp(it));
					break;
				default:
					std::cout << "Unknown GameObjType:" << (int)it.gameobjtype() << std::endl;
				}
			}
		}

		for (int i = 0; i < StateConstant::nCells; i++)
		{
			for (int j = 0; j < StateConstant::nCells; j++)
			{
				if (
#ifdef _ALL_VISIBLE_
					true
#else
					CellColorVisible(selfX, selfY, i, j)
#endif
					)
				{
#ifdef _COLOR_MAP_BY_HASHING_
					pBuffer->cellColors.insert(std::pair((i << 16) + j, (THUAI4::ColorType)pM2C->cellcolors(i).rowcolors(j)));
#else
					pBuffer->cellColors[i][j] = (THUAI4::ColorType)pM2C->cellcolors(i).rowcolors(j);
#endif // _COLOR_MAP_BY_HASHING_
				}
#ifndef _COLOR_MAP_BY_HASHING_
				//unorderer_map
				else
				{
					pBuffer->cellColors[i][j] = THUAI4::ColorType::Invisible;
				}
#endif // _COLOR_MAP_BY_HASHING_
			}
		}

		FlagBufferUpdated = true;
		counter_buffer += 1;

		//如果这时候state还没被player访问，就把buffer转到state
		if (mtx_state.try_lock())
		{
			State* temp = pState;
			pState = pBuffer;
			pBuffer = temp;
			FlagBufferUpdated = false;
			counter_state = counter_buffer;
			CurrentStateAccessed = false;
			mtx_state.unlock();
		}
	}
	cv_buffer.notify_one();
}

void Logic::UnBlockMtxOnReceive()
{
	{
		std::lock_guard<std::mutex> lck(mtxOnReceive);
		FlagProcessMessage = true;
	}
	cvOnReceive.notify_one();
}
void Logic::UnBlockMtxBufferUpdated()
{
	{
		std::lock_guard<std::mutex> lck(mtx_buffer);
		FlagBufferUpdated = true;
	}
	cv_buffer.notify_one();
}

void Logic::ProcessMessage()
{
	Pointer2Message p2M;
	while (gamePhase != GamePhase::GameOver)
	{
		//无消息处理时停下来少占资源
		{
			std::unique_lock<std::mutex> lck(mtxOnReceive); //OnReceive里往队列里Push时也锁了
			FlagProcessMessage = !queue.empty();
			cvOnReceive.wait(lck, [this]() { return FlagProcessMessage; });
		}

		if (!queue.try_pop(p2M))
		{
			if (gamePhase != GamePhase::GameOver) std::cout << "Failed to pop the message\n";
			continue;
		}

		//处理消息
		switch (p2M.index())
		{
		case 0: //M2C
			ProcessM2C(std::get<std::shared_ptr<Protobuf::MessageToClient>>(p2M));
			break;
		case 1: //M2OC
			ProcessM2OC(std::get<std::shared_ptr<Protobuf::MessageToOneClient>>(p2M));
			break;
		default:
			std::cout << "std::variant_nops\n";
		}
	}
	std::cout << "PM thread terminates" << std::endl;
}


void Logic::PlayerWrapper()
{
	{
		std::lock_guard<std::mutex> lock_ai(mtx_ai);
		AiTerminated = false;
	}
	while (gamePhase == GamePhase::Gaming)
	{
		std::lock_guard<std::mutex> lck_state(mtx_state);
		if (!CurrentStateAccessed)
		{
			pApi->StartTimer();
			pAI->play(*pApi);
			pApi->EndTimer();
			CurrentStateAccessed = true;
		}
		else
		{
			//否则看buffer是否有更新，更新的前提是buffer没被占用
			//所以这里堵塞是可以接受的
			std::unique_lock<std::mutex> lck_buffer(mtx_buffer);
			if (FlagBufferUpdated)
			{
				State* temp = pState;
				pState = pBuffer;
				pBuffer = temp;
				counter_state = counter_buffer;
				CurrentStateAccessed = false;
				FlagBufferUpdated = false;
			}
			else
			{ //如果当前state已经接触过且buffer没更新，那就等到buffer更新
				//意外断线这里也会锁住
				cv_buffer.wait(lck_buffer, [this]() { return FlagBufferUpdated; });
				State* temp = pState;
				pState = pBuffer;
				pBuffer = temp;
				counter_state = counter_buffer;
				CurrentStateAccessed = false;
				FlagBufferUpdated = false;
			}
		}
	}
	{
		std::lock_guard<std::mutex> lock_ai(mtx_ai);
		AiTerminated = true;
	}
	cv_ai.notify_one();
	std::cout << "AI thread terminates" << std::endl;
}

void Logic::PlayerWrapperAsyn()
{
	{
		std::lock_guard<std::mutex> lock_ai(mtx_ai);
		AiTerminated = false;
	}
	while (gamePhase == GamePhase::Gaming)
	{
		//异步似乎反而逻辑变简洁了
		pApi->StartTimer();
		pAI->play(*pApi);
		pApi->EndTimer();
	}
	{
		std::lock_guard<std::mutex> lock_ai(mtx_ai);
		AiTerminated = true;
	}
	cv_ai.notify_one();
	std::cout << "AI thread terminates" << std::endl;
}

void Logic::Main(const char* address, uint16_t port, int32_t playerID, int32_t teamID, THUAI4::JobType jobType, CreateAIFunc f, int debuglevel, std::string filename)
{
	this->playerID = playerID;
	this->teamID = teamID;
	this->jobType = jobType;
	this->pAI = f();

	std::ofstream OutFile;

	//又臭又长
	if (
		auto tu = [this]() {
			if (mtx_buffer.try_lock())
			{
				if (FlagBufferUpdated)
				{
					State* temp = pState;
						pState = pBuffer;
						pBuffer = temp;
						counter_state = counter_buffer;
						FlagBufferUpdated = false;
				}
				mtx_buffer.unlock();
			} };
	asynchronous)
	{
		if (!debuglevel)
		{
			this->pApi = std::make_unique<API<true>>([this](Protobuf::MessageToServer& M2C) {M2C.set_playerid(this->playerID); M2C.set_teamid(this->teamID); capi.Send(M2C); },
				[this]() { return MessageStorage.empty(); },
				[this](std::string& s) { return MessageStorage.try_pop(s); }, [this]() { return counter_state; },
				(const State*&)pState, mtx_state, tu);
		}
		else
		{
			bool flag = filename == "";
			if (!flag)
			{
				OutFile.open(filename);
				if (OutFile.fail())
				{
					std::cout << "Failed to open the file " << filename << std::endl;
					flag = true;
				}
			}
			this->pApi = std::make_unique<DebugApi<true>>([this](Protobuf::MessageToServer& M2C) {M2C.set_playerid(this->playerID); M2C.set_teamid(this->teamID); capi.Send(M2C); },
				[this]() { return MessageStorage.empty(); },
				[this](std::string& s) { return MessageStorage.try_pop(s); }, [this]() { return counter_state; },
				(const State*&)pState, mtx_state, tu, debuglevel != 1,
				flag ? std::cout : OutFile);
		}
	}
	else
	{
		if (!debuglevel)
		{
			this->pApi = std::make_unique<API<false>>([this](Protobuf::MessageToServer& M2C) {M2C.set_playerid(this->playerID); M2C.set_teamid(this->teamID); capi.Send(M2C); },
				[this]() { return MessageStorage.empty(); },
				[this](std::string& s) { return MessageStorage.try_pop(s); }, [this]() { return counter_state; },
				(const State*&)pState, mtx_state, tu);
		}
		else
		{
			bool flag = filename == "";
			if (!flag)
			{
				OutFile.open(filename);
				if (OutFile.fail())
				{
					std::cout << "Failed to open the file " << filename << std::endl;
					flag = true;
				}
			}
			this->pApi = std::make_unique<DebugApi<false>>([this](Protobuf::MessageToServer& M2C) {M2C.set_playerid(this->playerID); M2C.set_teamid(this->teamID); capi.Send(M2C); },
				[this]() { return MessageStorage.empty(); },
				[this](std::string& s) { return MessageStorage.try_pop(s); }, [this]() { return counter_state; },
				(const State*&)pState, mtx_state, tu, debuglevel != 1,
				flag ? std::cout : OutFile);
		}
	}

	std::thread tPM(&Logic::ProcessMessage, this); //单线程处理收到的消息
	if (!capi.Connect(address, port))
	{
		std::cout << "无法连接到Agent" << std::endl;
		capi.Stop();
		OutFile.close();
		tPM.join();
		return;
	}
	std::cout << "成功连接到Agent" << std::endl;

	tPM.join();

	UnBlockMtxBufferUpdated();//PlayerWrapper还可能在那卡着
	{
		std::unique_lock<std::mutex> lock(mtx_ai);
		cv_ai.wait(lock, [this]() { return AiTerminated; });
	}
	OutFile.close();
	capi.Stop();
}

Logic::Logic() : pState(storage), pBuffer(storage + 1),
capi(
	[this]()
	{
		Protobuf::MessageToServer message;
		message.set_messagetype(Protobuf::MessageType::AddPlayer);
		message.set_playerid(playerID);
		message.set_teamid(teamID);
		message.set_jobtype((Protobuf::JobType)jobType);
		capi.Send(message);
	},
	[this]()
	{
		std::cout << "Connection was closed.\n";
		gamePhase = GamePhase::GameOver;
		UnBlockMtxBufferUpdated();
		UnBlockMtxOnReceive();

	},
		[this](Pointer2Message p2M) {
		queue.push(p2M);
		UnBlockMtxOnReceive();
	})
{
}
